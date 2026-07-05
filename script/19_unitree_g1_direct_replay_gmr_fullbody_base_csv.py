#!/usr/bin/env python3
"""Replay a GMR Unitree G1 CSV directly with Unitree SDK2 Python.

This example bypasses TongRobot completely. It avoids the low-level
``rt/lowcmd`` full-body controller because that path can make the real robot
vibrate if the low-level gains/mode are not perfectly prepared.

Control path used here:

    base velocity       -> LocoClient.SetVelocity(...)
    upper-body joints   -> high-level rt/arm_sdk joint-angle command

Important: Unitree's high-level ``rt/arm_sdk`` example controls the upper body
only: left arm 7 + right arm 7 + waist 3 = 17 joints. The GMR CSV still has
29 joint columns, but this script intentionally ignores the 12 leg joints and
sends only upper-body joint angles. If you need true 29-joint leg control, that
requires the lower-level ``rt/lowcmd`` path or another Unitree API not present in
``unitree_sdk2_python-master/example/g1/high_level``.

Run on the robot PC, from this project root:

Dry-run from the CSV only. This sends no robot command and does not require DDS:

    python3 examples/19_unitree_g1_direct_replay_gmr_fullbody_base_csv.py

Dry-run using live robot upper-body state as the relative-motion reference:

    python3 examples/19_unitree_g1_direct_replay_gmr_fullbody_base_csv.py \
      --network-interface eth0 --use-live-state


Test upper-body joint angles only through rt/arm_sdk, without base velocity:

    python3 examples/19_unitree_g1_direct_replay_gmr_fullbody_base_csv.py \
      --network-interface eth0 \
      --disable-base \
      --use-live-state \
      --mode relative \
      --scale 1.0 \
      --max-joint-delta 12.56 \
      --smooth-window 10 \
      --blend-frames 50 \
      --max-frames 1000 \
      --control-hz 30 \
      --execute --yes



Add ``--csv PATH`` to override the default CSV. The expected CSV layout from
``GMR/scripts/run_video_to_g1.sh`` is:

    root_pos(3) + root_rot(4) + dof_pos(29)

Safety notes
------------
This script never calls Damp/Squat2StandUp and never uses ``rt/lowcmd``. It
uses Unitree's high-level arm SDK enable flag at motor_cmd[29].q. On exit it
ramps that flag back to 0 to release arm_sdk.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

DEFAULT_CSV = Path("speech_hs_2.csv")
# FALLBACK_CSV = Path("/home/zjw/workspace/gmr/GMR/unitree_g1_gmr/video1782695783507.csv")
FALLBACK_CSV = Path("/home/jingbohan/Projects/GMR/unitree_g1_gmr/speech_4.csv")
G1_NUM_MOTOR = 29
ARM_SDK_ENABLE_INDEX = 29

G1_JOINT_NAMES_29 = [
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
]

# Unitree high-level arm_sdk joint set: left arm 7 + right arm 7 + waist 3.
# Keep the motor indices exactly aligned with G1 lowstate / GMR 29-order.
# Waist joints are at indices 12,13,14 (waist_yaw, waist_roll, waist_pitch).
# When --disable-waist is used, these are excluded to avoid conflict with the
# G1 built-in balance controller (Loco).
WAIST_INDICES = (12, 13, 14)
UPPER_BODY_INDICES = np.asarray(
    [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 12, 13, 14],
    dtype=np.int32,
)


def upper_body_names() -> list[str]:
    return [G1_JOINT_NAMES_29[int(i)] for i in UPPER_BODY_INDICES]


@dataclass
class DirectUnitreeG1ArmSdk:
    network_interface: str

    def __post_init__(self) -> None:
        self.low_state: Any | None = None
        self.arm_cmd: Any | None = None
        self.arm_pub: Any | None = None
        self.loco: Any | None = None
        self.crc: Any | None = None
        self._handles: dict[str, Any] = {}

    def connect(self, need_arm_sdk: bool, need_loco: bool) -> None:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        print(f"Initializing Unitree DDS on interface: {self.network_interface}")
        ChannelFactoryInitialize(0, self.network_interface)
        self.crc = CRC()

        lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        lowstate_sub.Init(self._low_state_handler, 10)
        self._handles["lowstate_sub"] = lowstate_sub

        if need_arm_sdk:
            self.arm_cmd = unitree_hg_msg_dds__LowCmd_()
            self.arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self.arm_pub.Init()

        if need_loco:
            self.loco = LocoClient()
            self.loco.SetTimeout(10.0)
            self.loco.Init()

    def _low_state_handler(self, msg: Any) -> None:
        self.low_state = msg

    def wait_low_state(self, timeout_s: float) -> np.ndarray:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            q = self.get_current_q29_or_none()
            if q is not None:
                return q
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate")

    def get_current_q29_or_none(self) -> np.ndarray | None:
        st = self.low_state
        if st is None:
            return None
        try:
            return np.asarray([st.motor_state[i].q for i in range(G1_NUM_MOTOR)], dtype=float)
        except Exception:
            return None

    def send_base_velocity(self, vx: float, vy: float, wz: float, duration_s: float) -> None:
        if self.loco is None:
            raise RuntimeError("LocoClient is not initialized")
        code = self.loco.SetVelocity(float(vx), float(vy), float(wz), float(duration_s))
        if code != 0:
            raise RuntimeError(f"SetVelocity failed with code={code}")

    def stop_base(self) -> None:
        if self.loco is None:
            return
        for _ in range(2):
            try:
                self.send_base_velocity(0.0, 0.0, 0.0, 0.1)
            except Exception as exc:
                print(f"Warning: stop base failed: {exc}")
            time.sleep(0.02)

    def send_arm_sdk_q(
        self,
        q29: np.ndarray,
        dq29: np.ndarray,
        arm_kp: float,
        arm_kd: float,
        waist_kp: float,
        waist_kd: float,
        weight: float,
    ) -> None:
        if self.arm_cmd is None or self.arm_pub is None or self.crc is None:
            raise RuntimeError("arm_sdk publisher is not initialized")
        q29 = np.asarray(q29, dtype=float).reshape(G1_NUM_MOTOR)
        dq29 = np.asarray(dq29, dtype=float).reshape(G1_NUM_MOTOR)

        self.arm_cmd.motor_cmd[ARM_SDK_ENABLE_INDEX].q = float(np.clip(weight, 0.0, 1.0))
        for joint in UPPER_BODY_INDICES.tolist():
            mc = self.arm_cmd.motor_cmd[int(joint)]
            mc.tau = 0.0
            mc.q = float(q29[int(joint)])
            mc.dq = float(dq29[int(joint)])
            # weight=1 => arm_sdk FULLY owns the upper body, so the waist MUST be
            # actively held (kp>0) or the torso collapses. The waist uses its own
            # (higher) kp/kd; its q comes from the trajectory (0 with
            # --disable-waist, i.e. the standing pose).
            if int(joint) in WAIST_INDICES:
                mc.kp = float(waist_kp)
                mc.kd = float(waist_kd)
            else:
                mc.kp = float(arm_kp)
                mc.kd = float(arm_kd)

        self.arm_cmd.crc = self.crc.Crc(self.arm_cmd)
        ok = self.arm_pub.Write(self.arm_cmd)
        if not ok:
            raise RuntimeError("arm_sdk publish failed")

    def release_arm_sdk(self, steps: int = 25, period: float = 0.02) -> None:
        if self.arm_cmd is None or self.arm_pub is None or self.crc is None:
            return
        for i in range(max(1, int(steps))):
            weight = 1.0 - float(i + 1) / float(max(1, int(steps)))
            self.arm_cmd.motor_cmd[ARM_SDK_ENABLE_INDEX].q = float(np.clip(weight, 0.0, 1.0))
            self.arm_cmd.crc = self.crc.Crc(self.arm_cmd)
            try:
                self.arm_pub.Write(self.arm_cmd)
            except Exception as exc:
                print(f"Warning: release arm_sdk failed: {exc}")
                break
            time.sleep(max(0.0, float(period)))


def resolve_default_csv() -> Path:
    if DEFAULT_CSV.exists():
        return DEFAULT_CSV
    return FALLBACK_CSV


def ask_yes_no(prompt: str) -> bool:
    ans = input(f"{prompt} [y/N]: ").strip().lower()
    return ans in {"y", "yes"}


def load_gmr_csv(csv_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    data = np.loadtxt(csv_path, delimiter=",", dtype=float)
    if data.ndim == 1:
        data = data[None, :]
    if data.shape[1] != 36:
        raise ValueError(
            "GMR CSV must have 36 columns: root_pos(3)+root_rot(4)+dof_pos(29), "
            f"got shape={data.shape}"
        )
    return data[:, 0:3], data[:, 3:7], data[:, 7:36]


def quat_to_yaw(quat: np.ndarray, fmt: str) -> np.ndarray:
    q = np.asarray(quat, dtype=float)
    if q.ndim != 2 or q.shape[1] != 4:
        raise ValueError(f"root_rot must be (T,4), got {q.shape}")
    q = q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)
    if fmt == "xyzw":
        x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    elif fmt == "wxyz":
        w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    else:
        raise ValueError(f"Unsupported quaternion format: {fmt}")
    return np.unwrap(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def select_frames(
    root_pos: np.ndarray,
    root_rot: np.ndarray,
    dof29: np.ndarray,
    start_frame: int,
    max_frames: int | None,
    stride: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    stride = max(1, int(stride))
    start_frame = max(0, int(start_frame))
    stop = root_pos.shape[0] if max_frames is None else start_frame + int(max_frames)
    root_sel = root_pos[start_frame:stop:stride]
    rot_sel = root_rot[start_frame:stop:stride]
    dof_sel = dof29[start_frame:stop:stride]
    if root_sel.size == 0:
        raise ValueError("No frames selected from CSV")
    return root_sel, rot_sel, dof_sel


def moving_average(traj: np.ndarray, window: int) -> np.ndarray:
    window = int(window)
    if window <= 1:
        return traj
    if window % 2 == 0:
        window += 1
    pad = window // 2
    padded = np.pad(traj, ((pad, pad), (0, 0)), mode="edge")
    out = np.empty_like(traj)
    for i in range(traj.shape[0]):
        out[i] = np.mean(padded[i : i + window], axis=0)
    return out


def build_joint_trajectory(
    csv_q29: np.ndarray,
    live_q29: np.ndarray,
    mode: str,
    scale: float,
    max_joint_delta: float,
    blend_frames: int,
    disable_waist: bool = False,
) -> np.ndarray:
    live = np.asarray(live_q29, dtype=float).reshape(-1)
    if csv_q29.ndim != 2 or csv_q29.shape[1] != live.size:
        raise ValueError(f"csv/live mismatch: csv={csv_q29.shape}, live={live.shape}")

    cmd = csv_q29.copy() if mode == "absolute" else live[None, :] + (csv_q29 - csv_q29[0:1]) * scale
    if max_joint_delta > 0.0:
        cmd = np.clip(cmd, live[None, :] - max_joint_delta, live[None, :] + max_joint_delta)

    # When waist is disabled, lock waist joints to the live (current) position
    # for the entire trajectory — no motion, no blend. This avoids conflict
    # with the G1 built-in balance controller.
    if disable_waist:
        for j in WAIST_INDICES:
            # cmd[:, j] = live[j]
            cmd[:, j] = 0.

    blend_frames = max(0, min(int(blend_frames), cmd.shape[0]))
    if blend_frames > 0:
        target = cmd[:blend_frames].copy()
        for i in range(blend_frames):
            alpha = (i + 1) / float(blend_frames)
            cmd[i] = live * (1.0 - alpha) + target[i] * alpha
    return cmd


def build_joint_velocity(joint_traj: np.ndarray, period: float, max_dq: float) -> np.ndarray:
    dq = np.zeros_like(joint_traj)
    if joint_traj.shape[0] > 1:
        dq[:-1] = (joint_traj[1:] - joint_traj[:-1]) / period
        dq[-1] = dq[-2]
    if max_dq > 0.0:
        dq = np.clip(dq, -abs(max_dq), abs(max_dq))
    return dq


def build_base_velocity(
    root_pos: np.ndarray,
    root_rot: np.ndarray,
    fps: float,
    speed: float,
    quat_format: str,
    scale_xy: float,
    scale_yaw: float,
    max_vx: float,
    max_vy: float,
    max_wz: float,
) -> np.ndarray:
    pos = np.asarray(root_pos, dtype=float)
    yaw = quat_to_yaw(root_rot, quat_format)
    dt = 1.0 / max(1.0, fps * max(0.01, speed))

    dpos = np.zeros((pos.shape[0], 3), dtype=float)
    dyaw = np.zeros(pos.shape[0], dtype=float)
    if pos.shape[0] > 1:
        dpos[:-1] = pos[1:] - pos[:-1]
        dpos[-1] = dpos[-2]
        dyaw[:-1] = yaw[1:] - yaw[:-1]
        dyaw[-1] = dyaw[-2]

    vel_world = dpos[:, :2] / dt * scale_xy
    c = np.cos(yaw)
    s = np.sin(yaw)
    vx = c * vel_world[:, 0] + s * vel_world[:, 1]
    vy = -s * vel_world[:, 0] + c * vel_world[:, 1]
    wz = dyaw / dt * scale_yaw
    cmd = np.stack([vx, vy, wz], axis=1)
    cmd[:, 0] = np.clip(cmd[:, 0], -abs(max_vx), abs(max_vx))
    cmd[:, 1] = np.clip(cmd[:, 1], -abs(max_vy), abs(max_vy))
    cmd[:, 2] = np.clip(cmd[:, 2], -abs(max_wz), abs(max_wz))
    return cmd


def print_summary(
    root_pos: np.ndarray,
    joint_traj: np.ndarray,
    base_vel: np.ndarray,
    live_q29: np.ndarray,
    csv_q29: np.ndarray,
    disable_waist: bool = False,
) -> None:
    print("\nTrajectory summary:")
    print(f"  frames        : {joint_traj.shape[0]}")
    print(f"  root xyz min  : {np.min(root_pos, axis=0).round(4).tolist()}")
    print(f"  root xyz max  : {np.max(root_pos, axis=0).round(4).tolist()}")
    print(f"  base vel min  : {np.min(base_vel, axis=0).round(4).tolist()} [vx,vy,wz]")
    print(f"  base vel max  : {np.max(base_vel, axis=0).round(4).tolist()} [vx,vy,wz]")
    if disable_waist:
        print("  ** Waist JOINTS LOCKED to live position (CSV waist data ignored) **")
    print("\nUpper-body joint-angle command range sent to rt/arm_sdk:")
    print(
        f"{'idx':>3}  {'joint':<24} {'live/ref':>9} {'csv_min':>9} "
        f"{'csv_max':>9} {'cmd_min':>9} {'cmd_max':>9}"
    )
    csv_min = np.min(csv_q29, axis=0)
    csv_max = np.max(csv_q29, axis=0)
    cmd_min = np.min(joint_traj, axis=0)
    cmd_max = np.max(joint_traj, axis=0)
    for i in UPPER_BODY_INDICES.tolist():
        print(
            f"{i:>3}  {G1_JOINT_NAMES_29[i]:<24} {live_q29[i]:>9.4f} "
            f"{csv_min[i]:>9.4f} {csv_max[i]:>9.4f} "
            f"{cmd_min[i]:>9.4f} {cmd_max[i]:>9.4f}"
        )


def replay(
    robot: DirectUnitreeG1ArmSdk,
    joint_traj: np.ndarray,
    joint_dq: np.ndarray,
    base_vel: np.ndarray,
    control_hz: float,
    speed: float,
    enable_base: bool,
    enable_joints: bool,
    arm_kp: float,
    arm_kd: float,
    waist_kp: float,
    waist_kd: float,
) -> int:
    period = 1.0 / max(1.0, control_hz * max(0.01, speed))
    next_t = time.monotonic()
    sent = 0
    for frame_idx, (q, dq, vel) in enumerate(zip(joint_traj, joint_dq, base_vel)):
        if enable_base:
            robot.send_base_velocity(float(vel[0]), float(vel[1]), float(vel[2]), period * 2.0)
        if enable_joints:
            weight = min(1.0, float(frame_idx + 1) / 25.0)
            robot.send_arm_sdk_q(
                q, dq, arm_kp=arm_kp, arm_kd=arm_kd,
                waist_kp=waist_kp, waist_kd=waist_kd, weight=weight,
            )
        sent += 1
        next_t += period
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_t = time.monotonic()
    if enable_base:
        robot.stop_base()
    return sent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay GMR CSV directly on Unitree G1 via high-level arm_sdk + LocoClient."
    )
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--csv", type=Path, default=resolve_default_csv())
    parser.add_argument("--fps", type=float, default=50.0, help="Source CSV FPS.")
    parser.add_argument("--control-hz", type=float, default=50.0, help="Command loop rate.")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--quat-format", choices=["xyzw", "wxyz"], default="xyzw")
    parser.add_argument("--mode", choices=["relative", "absolute"], default="relative")
    parser.add_argument("--scale", type=float, default=1.0, help="Joint relative scale.")
    parser.add_argument("--max-joint-delta", type=float, default=0.5)
    parser.add_argument("--max-dq", type=float, default=4.0)
    parser.add_argument("--smooth-window", type=int, default=1)
    parser.add_argument("--blend-frames", type=int, default=25)
    parser.add_argument("--base-scale-xy", type=float, default=1.0)
    parser.add_argument("--base-scale-yaw", type=float, default=1.0)
    parser.add_argument("--max-vx", type=float, default=0.30)
    parser.add_argument("--max-vy", type=float, default=0.20)
    parser.add_argument("--max-wz", type=float, default=0.50)
    parser.add_argument("--arm-kp", type=float, default=60.0)
    parser.add_argument("--arm-kd", type=float, default=1.5)
    parser.add_argument(
        "--waist-kp",
        type=float,
        default=100.0,
        help="kp for the waist joints (default %(default)s — validated stable on the G1 in "
        "locomotion mode; weight=1 means arm_sdk fully owns the waist, so it must be actively "
        "held with real kp).",
    )
    parser.add_argument("--waist-kd", type=float, default=1.5)
    parser.add_argument("--disable-base", action="store_true")
    parser.add_argument("--disable-joints", action="store_true")
    parser.add_argument(
        "--disable-waist",
        action="store_true",
        help="Exclude waist joints (waist_yaw/roll/pitch) from arm_sdk to avoid "
        "conflict with the G1 built-in balance controller.",
    )
    parser.add_argument(
        "--use-live-state",
        action="store_true",
        help="Read current rt/lowstate q29 as the relative trajectory reference.",
    )
    parser.add_argument("--lowstate-timeout", type=float, default=5.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--yes", action="store_true")
    return parser.parse_args()


def run_gmr_direct_arm_sdk(
    network_interface: str = "eth0",
    csv: str | Path | None = None,
    fps: float = 50.0,
    control_hz: float = 50.0,
    speed: float = 1.0,
    start_frame: int = 0,
    max_frames: int | None = None,
    stride: int = 1,
    quat_format: str = "xyzw",
    mode: str = "relative",
    scale: float = 1.0,
    max_joint_delta: float = 0.5,
    max_dq: float = 4.0,
    smooth_window: int = 1,
    blend_frames: int = 25,
    base_scale_xy: float = 1.0,
    base_scale_yaw: float = 1.0,
    max_vx: float = 0.30,
    max_vy: float = 0.20,
    max_wz: float = 0.50,
    arm_kp: float = 60.0,
    arm_kd: float = 1.5,
    waist_kp: float = 100.0,
    waist_kd: float = 1.5,
    enable_base: bool = True,
    enable_joints: bool = True,
    disable_waist: bool = False,
    use_live_state: bool = False,
    lowstate_timeout: float = 5.0,
    execute: bool = False,
    yes: bool = False,
) -> int:
    """Run the direct Unitree G1 GMR replay flow.

    This is the callable version of ``main()``. By default it is a dry-run and
    sends no robot command. Set ``execute=True`` to command the real robot.

    Example::

        import importlib.util

        path = "examples/19_unitree_g1_direct_replay_gmr_fullbody_base_csv.py"
        spec = importlib.util.spec_from_file_location("g1_direct_replay", path)
        g1_direct_replay = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(g1_direct_replay)

        g1_direct_replay.run_gmr_direct_arm_sdk(
            network_interface="eth0",
            enable_base=False,
            use_live_state=True,
            max_frames=100,
            execute=True,
            yes=True,
        )
    """
    if not enable_base and not enable_joints:
        raise ValueError("Both base and joints are disabled; nothing to do")
    if quat_format not in {"xyzw", "wxyz"}:
        raise ValueError(f"Unsupported quat_format: {quat_format}")
    if mode not in {"relative", "absolute"}:
        raise ValueError(f"Unsupported mode: {mode}")

    csv_path = resolve_default_csv() if csv is None else Path(csv)
    root_pos, root_rot, dof29 = load_gmr_csv(csv_path)
    root_pos, root_rot, dof29 = select_frames(
        root_pos, root_rot, dof29, start_frame, max_frames, stride
    )
    dof29 = moving_average(dof29, smooth_window)

    # Resolve which joints are actually commanded — always all 17 upper-body
    # joints.  When --disable-waist the waist trajectory is locked to live
    # position in build_joint_trajectory(), so the arm_sdk still receives
    # valid motor_cmd entries for every joint (avoiding undefined behaviour
    # from uninitialised DDS fields) but the waist does not move.

    print(f"CSV: {csv_path}")
    print(f"Selected frames: {dof29.shape[0]}")
    print(f"Mode: {mode}, quat_format={quat_format}")
    print(f"Base enabled: {enable_base}, arm_sdk upper-body joints enabled: {enable_joints}")
    print(f"Waist disabled (locked to live position): {disable_waist}")
    print(f"Upper-body joints sent (17): {upper_body_names()}")

    needs_robot = execute or use_live_state
    robot: DirectUnitreeG1ArmSdk | None = None
    if needs_robot:
        robot = DirectUnitreeG1ArmSdk(network_interface)
        robot.connect(need_arm_sdk=enable_joints, need_loco=enable_base)
        live_q29 = robot.wait_low_state(lowstate_timeout)
        print(f"First 6 live q: {live_q29[:6].round(4).tolist()}")
    else:
        live_q29 = dof29[0].copy()
        print("No live state requested. Dry-run uses CSV first frame as joint reference.")

    joint_traj = build_joint_trajectory(
        dof29,
        live_q29,
        mode=mode,
        scale=scale,
        max_joint_delta=max_joint_delta,
        blend_frames=blend_frames,
        disable_waist=disable_waist,
    )
    base_vel = build_base_velocity(
        root_pos,
        root_rot,
        fps=fps,
        speed=speed,
        quat_format=quat_format,
        scale_xy=base_scale_xy,
        scale_yaw=base_scale_yaw,
        max_vx=max_vx,
        max_vy=max_vy,
        max_wz=max_wz,
    )
    period = 1.0 / max(1.0, control_hz * max(0.01, speed))
    joint_dq = build_joint_velocity(joint_traj, period, max_dq)

    print_summary(root_pos, joint_traj, base_vel, live_q29, dof29, disable_waist=disable_waist)
    duration = joint_traj.shape[0] * period
    print(f"\nEstimated playback duration: {duration:.2f}s")
    print(f"Command period: {period:.4f}s, control_hz={control_hz}")
    print(f"arm_sdk kp={arm_kp}, kd={arm_kd}")

    if not execute:
        print("Dry-run only. Add --execute or pass execute=True to command the real robot.")
        return 0
    if robot is None:
        raise RuntimeError("Internal error: robot is not connected")

    print("\nSafety checklist:")
    print("  [ ] This script bypasses TongRobot and commands Unitree SDK2 directly")
    print("  [ ] Joint command path is rt/arm_sdk, not rt/lowcmd")
    print("  [ ] Only upper-body 17 joints are commanded; leg joints from CSV are ignored")
    print("  [ ] Base velocity and upper-body motion have been tested separately first")
    print("  [ ] Operator can trigger hardware E-stop immediately")
    if not yes and not ask_yes_no("Execute direct Unitree high-level commands now?"):
        print("Aborted.")
        return 1

    try:
        sent = replay(
            robot,
            joint_traj,
            joint_dq,
            base_vel,
            control_hz=control_hz,
            speed=speed,
            enable_base=enable_base,
            enable_joints=enable_joints,
            arm_kp=arm_kp,
            arm_kd=arm_kd,
            waist_kp=waist_kp,
            waist_kd=waist_kd,
        )
        print(f"Sent {sent} synchronized frames.")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by operator.")
        return 130
    finally:
        if robot is not None:
            robot.stop_base()
            robot.release_arm_sdk()


def main() -> int:
    args = parse_args()
    return run_gmr_direct_arm_sdk(
        network_interface=args.network_interface,
        csv=args.csv,
        fps=args.fps,
        control_hz=args.control_hz,
        speed=args.speed,
        start_frame=args.start_frame,
        max_frames=args.max_frames,
        stride=args.stride,
        quat_format=args.quat_format,
        mode=args.mode,
        scale=args.scale,
        max_joint_delta=args.max_joint_delta,
        max_dq=args.max_dq,
        smooth_window=args.smooth_window,
        blend_frames=args.blend_frames,
        base_scale_xy=args.base_scale_xy,
        base_scale_yaw=args.base_scale_yaw,
        max_vx=args.max_vx,
        max_vy=args.max_vy,
        max_wz=args.max_wz,
        arm_kp=args.arm_kp,
        arm_kd=args.arm_kd,
        waist_kp=args.waist_kp,
        waist_kd=args.waist_kd,
        enable_base=not args.disable_base,
        enable_joints=not args.disable_joints,
        disable_waist=args.disable_waist,
        use_live_state=args.use_live_state,
        lowstate_timeout=args.lowstate_timeout,
        execute=args.execute,
        yes=args.yes,
    )


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"Error: {exc}")
        sys.exit(3)
