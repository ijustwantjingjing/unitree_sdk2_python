#!/usr/bin/env python3
"""Script 21 — Test GMR motion INTERRUPT + RESET-to-rest on Unitree G1.

Purpose
-------
Companion test for the upcoming gmr "interrupt" feature. It does two things:

    1. Plays a GMR CSV gesture on the robot (rt/arm_sdk, upper body only),
       exactly like script 19.
    2. INTERRUPTS the playback mid-gesture and RESETS both arms to a natural
       "arms at the sides" rest pose.

The reset is deliberately NOT another CSV replay. A CSV is a dense recorded
keyframe sequence; the reset has only TWO keyframes — the live pose at the
moment of interrupt (start) and the rest pose (end) — and this script synthesizes
every frame in between (see ``build_reset_trajectory`` / ``reset_to_rest``).

Control path & safety
---------------------
High-level ``rt/arm_sdk`` only (never ``rt/lowcmd``; never Damp/Squat2StandUp).
``motor_cmd[29].q`` is the arm_sdk enable WEIGHT:

    weight = 1  -> arm_sdk FULLY OWNS the upper body (14 arms + 3 waist); the
                   locomotion/balance controller has NO authority over those 17
                   joints. Therefore arm_sdk MUST actively hold every one of
                   them, especially the waist — never set waist kp=0 (the waist
                   would have no controller and the torso would collapse).
    weight = 0  -> the robot's internal controller resumes the upper body.

So this script always drives the 14 ARM joints with real kp/kd, and drives the
3 WAIST joints with their own kp/kd (default 60/1.5, the official arm_sdk
value, tunable via --waist-kp). The waist target is 0 by default (the standing
pose is ~0); use --waist-q live to write back the current angle instead.

IMPORTANT — run on a PC cabled to the G1 with the gmr service STOPPED, so the
two processes do not both publish rt/arm_sdk:

    sudo systemctl stop gmr        # on the G1

Then (from the unitree_sdk2_python project root), e.g.:

    python3 script/21_unitree_g1_gmr_interrupt_reset_arm_sdk.py \\
        --network-interface enx6c1ff7bbfb0c \\
        --csv /home/jingbohan/Projects/gmr/csv/speech_2.csv \\
        --play-frames 240 --execute --yes

At startup the script prints the EXACT parameters (arm kp/kd, waist kp/kd,
waist target q, reset duration, weight schedule) so you can correlate them
with what the robot does. Tune stability by raising --waist-kp (e.g. 100, 150)
and/or slowing the reset (--reset-frames). Too-high kp will make the upper
body vibrate — back off if it buzzes.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

G1_NUM_MOTOR = 29
ARM_SDK_ENABLE_INDEX = 29  # motor_cmd[29].q is the arm_sdk enable weight [0,1]

G1_JOINT_NAMES_29 = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
    "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
    "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]

# arm_sdk upper-body set: left arm 7 + right arm 7 + waist 3 (indices aligned
# with G1 lowstate / GMR 29-order). Waist = 12,13,14.
WAIST_INDICES = (12, 13, 14)
UPPER_BODY_INDICES = np.asarray(
    [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 12, 13, 14],
    dtype=np.int32,
)
# The 14 arm joints that the reset actually moves (left 15-21, right 22-28).
ARM_JOINT_INDICES = list(range(15, 29))

# Hardcoded fallback rest pose (rad) — only used with --rest-from-constant. The
# default is to CAPTURE the rest pose from live lowstate at startup (mode-
# agnostic; matches ready or locomotion standing pose). shoulder_roll /
# wrist_roll are mirrored (opposite sign) to match the G1 convention.
DEFAULT_REST_ARM_Q: dict[int, float] = {
    15: 0.29, 22: 0.29,    # shoulder_pitch
    16: 0.12, 23: -0.12,   # shoulder_roll (mirrored)
    17: 0.00, 24: 0.00,    # shoulder_yaw
    18: 0.98, 25: 0.98,    # elbow
    19: 0.08, 26: -0.11,   # wrist_roll (mirrored)
    20: 0.04, 27: 0.04,    # wrist_pitch
    21: 0.00, 28: 0.00,    # wrist_yaw
}


# ---------------------------------------------------------------------------
# Robot connection — rt/arm_sdk publisher + rt/lowstate subscriber
# ---------------------------------------------------------------------------

@dataclass
class DirectUnitreeG1ArmSdk:
    network_interface: str

    def __post_init__(self) -> None:
        self.low_state: Any | None = None
        self.arm_cmd: Any | None = None
        self.arm_pub: Any | None = None
        self.crc: Any | None = None
        # Last published enable weight. The reset uses this so it can take over
        # smoothly after an interrupt (weight already 1.0) instead of restarting
        # from 0 and briefly dropping the upper body.
        self.cur_weight: float = 0.0
        self._handles: dict[str, Any] = {}

    def connect(self) -> None:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        print(f"Initializing Unitree DDS on interface: {self.network_interface}")
        ChannelFactoryInitialize(0, self.network_interface)
        self.crc = CRC()

        lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        lowstate_sub.Init(self._low_state_handler, 10)
        self._handles["lowstate_sub"] = lowstate_sub

        self.arm_cmd = unitree_hg_msg_dds__LowCmd_()
        self.arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_pub.Init()

    def _low_state_handler(self, msg: Any) -> None:
        self.low_state = msg

    def get_current_q29_or_none(self) -> np.ndarray | None:
        st = self.low_state
        if st is None:
            return None
        try:
            return np.asarray([st.motor_state[i].q for i in range(G1_NUM_MOTOR)], dtype=float)
        except Exception:
            return None

    def wait_low_state(self, timeout_s: float) -> np.ndarray:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            q = self.get_current_q29_or_none()
            if q is not None:
                return q
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate")

    def send_arm_sdk_q(
        self,
        q29: np.ndarray,
        dq29: np.ndarray,
        arm_kp: float,
        arm_kd: float,
        waist_kp: float,
        waist_kd: float,
        weight: float,
        waist_q_mode: str = "zero",
    ) -> None:
        """Publish one rt/arm_sdk frame.

        weight=1 means arm_sdk FULLY owns the upper body, so every upper-body
        joint (incl. waist) MUST get real kp/kd — never 0, or that joint has no
        controller.

          - 14 ARM joints: position-controlled with arm_kp/arm_kd.
          - 3 WAIST joints (12,13,14): position-controlled with waist_kp/waist_kd.
            The waist target q is chosen by waist_q_mode:
              'zero' : q = 0          (standing waist ~0; the "全部为0" method)
              'live' : q = current    (write back live lowstate each frame)
              'hold' : q = trajectory (the passed q29; held at the start value)
        """
        if self.arm_cmd is None or self.arm_pub is None or self.crc is None:
            raise RuntimeError("arm_sdk publisher is not initialized")
        q29 = np.asarray(q29, dtype=float).reshape(G1_NUM_MOTOR)
        dq29 = np.asarray(dq29, dtype=float).reshape(G1_NUM_MOTOR)
        w = float(np.clip(weight, 0.0, 1.0))

        live_now = None
        if waist_q_mode == "live":
            live_now = self.get_current_q29_or_none()

        self.arm_cmd.motor_cmd[ARM_SDK_ENABLE_INDEX].q = w
        for joint in UPPER_BODY_INDICES.tolist():
            mc = self.arm_cmd.motor_cmd[int(joint)]
            if int(joint) in WAIST_INDICES:
                if waist_q_mode == "live" and live_now is not None:
                    q_t = float(live_now[int(joint)])
                elif waist_q_mode == "hold":
                    q_t = float(q29[int(joint)])
                else:  # "zero"
                    q_t = 0.0
                mc.tau = 0.0
                mc.q = q_t
                mc.dq = 0.0
                mc.kp = float(waist_kp)
                mc.kd = float(waist_kd)
            else:  # arm joint
                mc.tau = 0.0
                mc.q = float(q29[int(joint)])
                mc.dq = float(dq29[int(joint)])
                mc.kp = float(arm_kp)
                mc.kd = float(arm_kd)
        self.arm_cmd.crc = self.crc.Crc(self.arm_cmd)
        if not self.arm_pub.Write(self.arm_cmd):
            raise RuntimeError("arm_sdk publish failed")
        self.cur_weight = w

    def release_arm_sdk(self, steps: int = 25, period: float = 0.02) -> None:
        if self.arm_cmd is None or self.arm_pub is None or self.crc is None:
            return
        for i in range(max(1, int(steps))):
            weight = 1.0 - float(i + 1) / float(max(1, int(steps)))
            self.arm_cmd.motor_cmd[ARM_SDK_ENABLE_INDEX].q = float(np.clip(weight, 0.0, 1.0))
            self.arm_cmd.crc = self.crc.Crc(self.arm_cmd)
            try:
                self.arm_pub.Write(self.arm_cmd)
            except Exception as exc:  # noqa: BLE001
                print(f"Warning: release arm_sdk failed: {exc}")
                break
            time.sleep(max(0.0, float(period)))
        self.cur_weight = 0.0


# ---------------------------------------------------------------------------
# CSV helpers (for the pre-interrupt gesture) — ported from script 19
# ---------------------------------------------------------------------------

def load_gmr_csv(csv_path: Path) -> np.ndarray:
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
    return data[:, 7:36]  # dof_pos(29) only — we replay upper body, ignore root


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
        out[i] = np.mean(padded[i: i + window], axis=0)
    return out


def build_joint_trajectory(
    csv_q29: np.ndarray,
    live_q29: np.ndarray,
    mode: str,
    scale: float,
    max_joint_delta: float,
    blend_frames: int,
    disable_waist: bool = True,
) -> np.ndarray:
    """Relative upper-body trajectory from the CSV, clamped + blended to live,
    with the waist locked to the live pose. Same logic as script 19. (The waist
    column here only matters under --waist-q hold.)"""
    live = np.asarray(live_q29, dtype=float).reshape(-1)
    cmd = csv_q29.copy() if mode == "absolute" else live[None, :] + (csv_q29 - csv_q29[0:1]) * scale
    if max_joint_delta > 0.0:
        cmd = np.clip(cmd, live[None, :] - max_joint_delta, live[None, :] + max_joint_delta)
    if disable_waist:
        for j in WAIST_INDICES:
            cmd[:, j] = live[j]
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


# ---------------------------------------------------------------------------
# RESET motion — the new functionality (NOT a CSV)
# ---------------------------------------------------------------------------

def smoothstep(t: float) -> float:
    """Cubic ease-in/out on [0,1]: 0 at t=0, 1 at t=1, zero velocity at both
    ends — a gentle point-to-point ramp with no jerk at start/finish."""
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


def build_rest_target_q29(live_q29: np.ndarray, rest_arm_q: dict[int, float]) -> np.ndarray:
    """Target pose = current pose with the 14 arm joints overwritten by the rest
    pose. Waist + legs stay at live."""
    target = np.asarray(live_q29, dtype=float).copy()
    for idx, val in rest_arm_q.items():
        target[int(idx)] = float(val)
    return target


def build_reset_trajectory(
    start_q29: np.ndarray,
    target_q29: np.ndarray,
    frames: int,
    period: float,
    max_dq: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Synthesize a smooth start->target trajectory (NOT from a CSV). Only the
    arm joints differ, so legs + waist are constant. The waist column matters
    only under --waist-q hold; under zero/live it is overridden in send_arm_sdk_q.
    Velocity is the finite difference of the eased position, clamped by max_dq."""
    frames = max(2, int(frames))
    traj = np.empty((frames, G1_NUM_MOTOR), dtype=float)
    for i in range(frames):
        alpha = smoothstep(i / (frames - 1))
        traj[i] = start_q29 * (1.0 - alpha) + target_q29 * alpha
    dq = np.zeros_like(traj)
    dq[:-1] = (traj[1:] - traj[:-1]) / period
    dq[-1] = dq[-2]
    if max_dq > 0.0:
        dq = np.clip(dq, -abs(max_dq), abs(max_dq))
    return traj, dq


def reset_to_rest(
    robot: DirectUnitreeG1ArmSdk,
    rest_arm_q: dict[int, float],
    frames: int,
    hz: float,
    arm_kp: float,
    arm_kd: float,
    waist_kp: float,
    waist_kd: float,
    max_dq: float,
    waist_q_mode: str = "zero",
    ramp_frames: int = 25,
) -> None:
    """Drive both arms from their current pose to the rest pose via rt/arm_sdk.
    Reads the live pose at call time, builds start->rest, publishes frame by
    frame. Weight is ramped from its current value up to 1.0 (engage); the caller
    releases it back to 0 in its finally-block."""
    period = 1.0 / max(1.0, float(hz))
    start_q = robot.wait_low_state(2.0)
    target_q = build_rest_target_q29(start_q, rest_arm_q)
    traj, dq = build_reset_trajectory(start_q, target_q, frames, period, max_dq)

    moved = [int(i) for i in ARM_JOINT_INDICES if abs(target_q[i] - start_q[i]) > 1e-3]
    print(
        f"  reset: {frames} frames / {frames * period:.2f}s @ {hz}Hz, "
        f"moving {len(moved)} arm joint(s); e.g. elbow "
        f"L {start_q[18]:+.3f}->{target_q[18]:+.3f}, "
        f"R {start_q[25]:+.3f}->{target_q[25]:+.3f}"
    )

    start_weight = robot.cur_weight
    next_t = time.monotonic()
    for i in range(frames):
        frac = min(1.0, float(i + 1) / max(1, ramp_frames))
        weight = start_weight + (1.0 - start_weight) * frac
        robot.send_arm_sdk_q(
            traj[i], dq[i],
            arm_kp=arm_kp, arm_kd=arm_kd,
            waist_kp=waist_kp, waist_kd=waist_kd,
            weight=weight, waist_q_mode=waist_q_mode,
        )
        next_t += period
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_t = time.monotonic()


# ---------------------------------------------------------------------------
# Interruptible playback
# ---------------------------------------------------------------------------

def play_until_interrupt(
    robot: DirectUnitreeG1ArmSdk,
    joint_traj: np.ndarray,
    joint_dq: np.ndarray,
    hz: float,
    arm_kp: float,
    arm_kd: float,
    waist_kp: float,
    waist_kd: float,
    interrupt_event: threading.Event,
    waist_q_mode: str = "zero",
) -> tuple[int, bool]:
    """Play the gesture frame by frame; stop the instant interrupt_event is set.
    Returns (frames_sent, was_interrupted). Checks the event every frame, so the
    interrupt latency is one control period (~33 ms at 30 Hz)."""
    period = 1.0 / max(1.0, float(hz))
    next_t = time.monotonic()
    sent = 0
    for i, (q, dq) in enumerate(zip(joint_traj, joint_dq)):
        if interrupt_event.is_set():
            break
        weight = min(1.0, float(i + 1) / 25.0)  # engage over the first 25 frames
        robot.send_arm_sdk_q(
            q, dq,
            arm_kp=arm_kp, arm_kd=arm_kd,
            waist_kp=waist_kp, waist_kd=waist_kd,
            weight=weight, waist_q_mode=waist_q_mode,
        )
        sent += 1
        next_t += period
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_t = time.monotonic()
    return sent, interrupt_event.is_set()


def start_interrupt_watcher(
    interrupt_event: threading.Event,
    mode: str,
    auto_frames: int,
    hz: float,
) -> threading.Thread:
    """Fire the interrupt after auto_frames (mode='auto') or on stdin Enter."""
    period = 1.0 / max(1.0, float(hz))

    def _watch() -> None:
        if mode == "enter":
            print("  (press Enter to interrupt)")
            try:
                sys.stdin.readline()
            except Exception:  # noqa: BLE001
                pass
        else:
            time.sleep(max(0.0, auto_frames * period))
        interrupt_event.set()

    t = threading.Thread(target=_watch, name="interrupt-watcher", daemon=True)
    t.start()
    return t


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def ask_yes_no(prompt: str) -> bool:
    return input(f"{prompt} [y/N]: ").strip().lower() in {"y", "yes"}


def print_rest_plan(live_q29: np.ndarray, rest_arm_q: dict[int, float]) -> None:
    print("\nArm reset plan (startup live -> rest target), rad:")
    print(f"{'idx':>3}  {'joint':<22} {'live':>9} {'rest':>9} {'delta':>9}")
    for idx in ARM_JOINT_INDICES:
        lv = float(live_q29[idx])
        rv = float(rest_arm_q[idx])
        print(f"{idx:>3}  {G1_JOINT_NAMES_29[idx]:<22} {lv:>9.3f} {rv:>9.3f} {rv - lv:>+9.3f}")


def print_parameters(args: argparse.Namespace, rest_source: str) -> None:
    print("\n=== Parameters (exact values being applied) ===")
    print(f"  arms  (14 joints, 15-28): kp={args.arm_kp}  kd={args.arm_kd}")
    print(f"  waist (3 joints, 12-14) : kp={args.waist_kp}  kd={args.waist_kd}  "
          f"q_target={args.waist_q}"
          + (" (=0)" if args.waist_q == "zero" else
             (" (=live lowstate)" if args.waist_q == "live" else " (=held start)")))
    print(f"  reset : {args.reset_frames} frames / {args.reset_frames / args.reset_hz:.2f}s "
          f"@ {args.reset_hz}Hz")
    print(f"  rest target: {rest_source}")
    print(f"  weight: ramp 0->1 over 25 frames (engage), release 1->0 over 25 frames")
    print(f"  NOTE: weight=1 => arm_sdk FULLY owns the upper body (arms + waist);")
    print(f"        the balance controller cannot help the waist while engaged.")
    print(f"  joint PD law: tau = kp*(q_target-q) + kd*(dq_target-dq) + tau_ff")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Test GMR interrupt + arm reset-to-rest on Unitree G1 (rt/arm_sdk)."
    )
    p.add_argument("--network-interface", default="eth0")
    p.add_argument("--csv", type=Path, default=None,
                   help="GMR CSV to play before the interrupt (omit for reset-only).")
    # pre-interrupt playback
    p.add_argument("--play-frames", type=int, default=240,
                   help="Auto-interrupt the gesture after this many frames (default %(default)s).")
    p.add_argument("--wait-enter", action="store_true",
                   help="Ignore --play-frames; play until Enter is pressed.")
    p.add_argument("--max-frames", type=int, default=1000)
    p.add_argument("--smooth-window", type=int, default=10)
    p.add_argument("--blend-frames", type=int, default=50)
    p.add_argument("--max-joint-delta", type=float, default=12.56)
    p.add_argument("--control-hz", type=float, default=30.0, help="Gesture loop rate.")
    # reset
    p.add_argument("--no-reset", action="store_true",
                   help="Play the gesture but do NOT reset afterwards (for comparison).")
    p.add_argument("--reset-frames", type=int, default=120,
                   help="Reset move length in frames (default %(default)s ≈ 4s @30Hz). "
                        "Longer = gentler; increase if the robot steps during reset.")
    p.add_argument("--reset-hz", type=float, default=30.0, help="Reset loop rate.")
    p.add_argument("--max-dq", type=float, default=4.0)
    # rest pose
    p.add_argument("--rest-from-constant", action="store_true",
                   help="Use the hardcoded DEFAULT_REST_ARM_Q instead of the live-captured home pose.")
    p.add_argument("--rest-shoulder-pitch", type=float, default=DEFAULT_REST_ARM_Q[15],
                   help="Only with --rest-from-constant.")
    p.add_argument("--rest-elbow", type=float, default=DEFAULT_REST_ARM_Q[18],
                   help="Only with --rest-from-constant.")
    # gains — arms vs waist, independently tunable (official arm_sdk value = 60/1.5)
    p.add_argument("--arm-kp", type=float, default=60.0,
                   help="kp for the 14 ARM joints (default %(default)s, official arm_sdk).")
    p.add_argument("--arm-kd", type=float, default=1.5,
                   help="kd for the 14 ARM joints (default %(default)s, official arm_sdk).")
    p.add_argument("--waist-kp", type=float, default=100.0,
                   help="kp for the 3 WAIST joints (default %(default)s — validated stable on the "
                        "G1 in locomotion mode). Lower if the torso vibrates; raise if it tilts.")
    p.add_argument("--waist-kd", type=float, default=1.5,
                   help="kd for the 3 WAIST joints (default %(default)s).")
    p.add_argument("--waist-q", choices=["zero", "live", "hold"], default="zero",
                   help="Waist target position q each frame. 'zero' (default): q=0 (standing waist "
                        "~0; the '全部为0' method). 'live': write back current waist angle. 'hold': "
                        "hold the start value. kp/kd are ALWAYS applied (never 0).")
    # misc
    p.add_argument("--lowstate-timeout", type=float, default=5.0)
    p.add_argument("--execute", action="store_true",
                   help="Command the real robot (default is a dry-run that only reads state).")
    p.add_argument("--yes", action="store_true")
    return p.parse_args()


def run(args: argparse.Namespace) -> int:
    robot = DirectUnitreeG1ArmSdk(args.network_interface)
    robot.connect()
    live_q = robot.wait_low_state(args.lowstate_timeout)
    print(f"Live q (first 6): {live_q[:6].round(4).tolist()}")

    # Reset target = natural standing pose captured at startup (mode-agnostic),
    # unless --rest-from-constant.
    if args.rest_from_constant:
        rest_arm_q = dict(DEFAULT_REST_ARM_Q)
        rest_arm_q[15] = rest_arm_q[22] = args.rest_shoulder_pitch
        rest_arm_q[18] = rest_arm_q[25] = args.rest_elbow
        rest_source = "hardcoded DEFAULT_REST_ARM_Q (--rest-from-constant)"
    else:
        rest_arm_q = {idx: float(live_q[idx]) for idx in ARM_JOINT_INDICES}
        rest_source = "captured from live lowstate at startup (natural pose)"
    print(f"Rest target: {rest_source}.")
    print_parameters(args, rest_source)
    print_rest_plan(live_q, rest_arm_q)

    if not args.execute:
        print("\nDry-run only — no commands sent. Add --execute to drive the robot.")
        return 0

    if args.csv is not None and not args.csv.is_file():
        print(f"\nCSV not found: {args.csv} — continuing in reset-only mode.")
        args.csv = None

    print("\nSafety checklist:")
    print("  [ ] gmr service stopped on the G1 (no two rt/arm_sdk owners)")
    print("  [ ] Robot standing naturally, clear space, operator can E-stop / damping mode")
    print("  [ ] weight=1 fully owns upper body — waist held by arm_sdk (kp>0), not balance")
    if not args.yes and not ask_yes_no("Execute now?"):
        print("Aborted.")
        return 1

    interrupt_event = threading.Event()
    try:
        if args.csv is not None:
            dof29 = load_gmr_csv(args.csv)
            if args.max_frames is not None:
                dof29 = dof29[: args.max_frames]
            dof29 = moving_average(dof29, args.smooth_window)
            joint_traj = build_joint_trajectory(
                dof29, live_q, mode="relative", scale=1.0,
                max_joint_delta=args.max_joint_delta,
                blend_frames=args.blend_frames, disable_waist=True,
            )
            joint_dq = build_joint_velocity(joint_traj, 1.0 / args.control_hz, args.max_dq)

            mode = "enter" if args.wait_enter else "auto"
            if mode == "auto":
                print(f"\nPlaying gesture; auto-interrupt after {args.play_frames} frames "
                      f"(~{args.play_frames / args.control_hz:.1f}s).")
            else:
                print("\nPlaying gesture.")
            start_interrupt_watcher(interrupt_event, mode, args.play_frames, args.control_hz)

            sent, interrupted = play_until_interrupt(
                robot, joint_traj, joint_dq, args.control_hz,
                args.arm_kp, args.arm_kd, args.waist_kp, args.waist_kd,
                interrupt_event, args.waist_q,
            )
            print(f"Gesture {'INTERRUPTED' if interrupted else 'completed'} after {sent} frames.")
        else:
            print("\nNo --csv given: reset-only mode (no gesture played first).")

        if args.no_reset:
            print("\n--no-reset: leaving arms where they are (no reset).")
        else:
            print("\nResetting arms to rest pose...")
            reset_to_rest(
                robot, rest_arm_q, args.reset_frames, args.reset_hz,
                args.arm_kp, args.arm_kd, args.waist_kp, args.waist_kd,
                args.max_dq, args.waist_q,
            )
            print("Reset complete.")
    except KeyboardInterrupt:
        print("\nInterrupted by operator.")
    finally:
        # Always hand the upper body back to the internal controller cleanly.
        robot.release_arm_sdk()
    return 0


def main() -> int:
    return run(parse_args())


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001
        print(f"Error: {exc}")
        sys.exit(3)
