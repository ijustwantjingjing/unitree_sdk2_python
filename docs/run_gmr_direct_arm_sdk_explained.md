# `run_gmr_direct_arm_sdk` 函数逐行详解

## 概述

`run_gmr_direct_arm_sdk` 是脚本 `19_unitree_g1_direct_replay_gmr_fullbody_base_csv.py` 的核心函数，位于第 510 行。它的作用是将 GMR（General Motion Replay）CSV 文件中的运动数据回放到宇树 G1 机器人上。控制路径分为两条：

- **底盘速度** → 通过 `LocoClient.SetVelocity()` 发送
- **上肢关节角度** → 通过高层 `rt/arm_sdk` DDS 通道发送（左臂7关节 + 右臂7关节 + 腰部3关节 = 共17个关节）

> 注意：该脚本使用高层 arm_sdk 而非底层 `rt/lowcmd`，后者如果增益/模式配置不当可能导致机器人抖动。

---

## 函数签名（第 510–540 行）

```python
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
    enable_base: bool = True,
    enable_joints: bool = True,
    disable_waist: bool = False,
    use_live_state: bool = False,
    lowstate_timeout: float = 5.0,
    execute: bool = False,
    yes: bool = False,
) -> int:
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `network_interface` | `str` | `"eth0"` | DDS 通信使用的网卡接口名 |
| `csv` | `str/Path/None` | `None` | CSV 文件路径，为 None 时使用默认路径 |
| `fps` | `float` | `50.0` | 源 CSV 的帧率 |
| `control_hz` | `float` | `50.0` | 控制指令发送频率 |
| `speed` | `float` | `1.0` | 回放速度倍率（>1 加速，<1 减速） |
| `start_frame` | `int` | `0` | 起始帧索引 |
| `max_frames` | `int/None` | `None` | 最大回放帧数，None 表示全部 |
| `stride` | `int` | `1` | 帧跳跃步长（>1 时跳帧） |
| `quat_format` | `str` | `"xyzw"` | 四元数格式，可选 `"xyzw"` 或 `"wxyz"` |
| `mode` | `str` | `"relative"` | 关节轨迹模式：`"relative"`（相对）或 `"absolute"`（绝对） |
| `scale` | `float` | `1.0` | 相对模式下关节运动的缩放因子 |
| `max_joint_delta` | `float` | `0.5` | 相对模式下关节最大偏移量（弧度） |
| `max_dq` | `float` | `4.0` | 关节速度上限（弧度/秒） |
| `smooth_window` | `int` | `1` | 平滑窗口大小（1 表示不平滑） |
| `blend_frames` | `int` | `25` | 从当前姿态过渡到目标轨迹的混合帧数 |
| `base_scale_xy` | `float` | `1.0` | 底盘 XY 速度缩放 |
| `base_scale_yaw` | `float` | `1.0` | 底盘偏航速度缩放 |
| `max_vx` | `float` | `0.30` | 底盘 X 方向最大速度（m/s） |
| `max_vy` | `float` | `0.20` | 底盘 Y 方向最大速度（m/s） |
| `max_wz` | `float` | `0.50` | 底盘最大偏航角速度（rad/s） |
| `arm_kp` | `float` | `60.0` | 上肢关节 PD 控制器的比例增益 |
| `arm_kd` | `float` | `1.5` | 上肢关节 PD 控制器的微分增益 |
| `enable_base` | `bool` | `True` | 是否发送底盘速度指令 |
| `enable_joints` | `bool` | `True` | 是否发送上肢关节角度指令 |
| `disable_waist` | `bool` | `False` | 是否锁定腰部关节（避免与 G1 内置平衡控制器冲突） |
| `use_live_state` | `bool` | `False` | 是否从机器人读取当前关节角度作为相对参考 |
| `lowstate_timeout` | `float` | `5.0` | 等待机器人低状态超时时间（秒） |
| `execute` | `bool` | `False` | 是否实际向机器人发送指令（False = 仅演练） |
| `yes` | `bool` | `False` | 是否跳过安全检查确认 |
| **返回值** | `int` | — | `0` 成功，非零表示错误或中止 |

---

## 逐行解析

### 第一部分：参数校验（第 564–569 行）

```python
if not enable_base and not enable_joints:
    raise ValueError("Both base and joints are disabled; nothing to do")
```

**作用**：检查是否底盘和关节都被禁用。如果两者都为 `False`，则没有任何指令可以发送，直接抛出异常。这是一个防御性检查，防止无意义的调用。

```python
if quat_format not in {"xyzw", "wxyz"}:
    raise ValueError(f"Unsupported quat_format: {quat_format}")
```

**作用**：验证 `quat_format` 参数。只支持 `"xyzw"` 和 `"wxyz"` 两种四元数分量排列方式。`"xyzw"` 表示四元数的 x, y, z, w 分量按此顺序存储；`"wxyz"` 表示 w, x, y, z 分量按此顺序存储。

```python
if mode not in {"relative", "absolute"}:
    raise ValueError(f"Unsupported mode: {mode}")
```

**作用**：验证 `mode` 参数。`"relative"` 表示关节轨迹是相对于当前姿态的偏移量；`"absolute"` 表示关节轨迹是绝对角度值。

---

### 第二部分：加载 CSV 数据（第 571–576 行）

```python
csv_path = resolve_default_csv() if csv is None else Path(csv)
```

**作用**：确定 CSV 文件的路径。如果调用者没有指定 `csv` 参数，则调用 `resolve_default_csv()`（第 247 行）查找默认文件。

**`resolve_default_csv()` 详解（第 247–250 行）**：

```python
def resolve_default_csv() -> Path:
    if DEFAULT_CSV.exists():
        return DEFAULT_CSV
    return FALLBACK_CSV
```

1. 首先检查当前目录下的 `speech_hs_2.csv` 是否存在（`DEFAULT_CSV = Path("speech_hs_2.csv")`，第 72 行）
2. 如果不存在，回退到绝对路径 `FALLBACK_CSV`（第 74 行）：
   `/home/jingbohan/Projects/GMR/unitree_g1_gmr/speech_hs_2.csv`
3. 这个设计使得脚本既可以在项目目录下运行，也可以在任意位置运行

---

```python
root_pos, root_rot, dof29 = load_gmr_csv(csv_path)
```

**作用**：调用 `load_gmr_csv()` 从 CSV 文件中加载运动数据。

**`load_gmr_csv()` 详解（第 258–269 行）**：

```python
def load_gmr_csv(csv_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
```

1. **第 259 行** `if not csv_path.is_file():` — 检查文件是否存在，不存在则抛出 `FileNotFoundError`
2. **第 261 行** `data = np.loadtxt(csv_path, delimiter=",", dtype=float)` — 使用 NumPy 读取逗号分隔的 CSV 文件，所有数据转换为浮点数。`np.loadtxt` 将文件读入一个二维数组，每行是一帧，每列是一个数据通道。
3. **第 262–263 行** `if data.ndim == 1: data = data[None, :]` — 如果只有一行数据（一维数组），则增加一个维度使其变为 `(1, N)` 形状的二维数组，统一后续处理逻辑。
4. **第 264–268 行** 验证列数必须是 36：
   - 列 0–2：`root_pos` — 根节点（机器人底座）的 XYZ 世界位置（3 列）
   - 列 3–6：`root_rot` — 根节点的旋转四元数（4 列）
   - 列 7–35：`dof_pos` — 29 个关节的角度位置（29 列）
5. **第 269 行** 返回三元组：`(root_pos, root_rot, dof29)`
   - `data[:, 0:3]` → `root_pos`，形状 `(T, 3)`
   - `data[:, 3:7]` → `root_rot`，形状 `(T, 4)`
   - `data[:, 7:36]` → `dof29`，形状 `(T, 29)`

---

```python
root_pos, root_rot, dof29 = select_frames(
    root_pos, root_rot, dof29, start_frame, max_frames, stride
)
```

**作用**：调用 `select_frames()` 从完整轨迹中选择要回放的帧。

**`select_frames()` 详解（第 286–302 行）**：

```python
def select_frames(
    root_pos: np.ndarray,
    root_rot: np.ndarray,
    dof29: np.ndarray,
    start_frame: int,
    max_frames: int | None,
    stride: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
```

1. **第 294 行** `stride = max(1, int(stride))` — 确保步长至少为 1，步长为 N 表示每隔 N-1 帧取一帧
2. **第 295 行** `start_frame = max(0, int(start_frame))` — 确保起始帧不小于 0
3. **第 296 行** `stop = root_pos.shape[0] if max_frames is None else start_frame + int(max_frames)` — 计算结束帧索引。`max_frames=None` 表示取到末尾；否则取 `start_frame + max_frames` 帧
4. **第 297–299 行** 使用 Python 切片 `[start:stop:stride]` 分别对位置、旋转、关节数据取子集
5. **第 300–301 行** 如果选择的帧数为空，抛出异常

**返回**：切片后的 `(root_pos, root_rot, dof29)` 三元组

---

```python
dof29 = moving_average(dof29, smooth_window)
```

**作用**：调用 `moving_average()` 对关节角度轨迹进行平滑处理，去除高频噪声。

**`moving_average()` 详解（第 305–316 行）**：

```python
def moving_average(traj: np.ndarray, window: int) -> np.ndarray:
```

1. **第 306–307 行** `window = int(window)`，如果 `window <= 1` 直接返回原数组（无需平滑）
2. **第 309–310 行** `if window % 2 == 0: window += 1` — 确保窗口大小为奇数，便于对称填充
3. **第 311 行** `pad = window // 2` — 计算每侧填充量（如 window=5 时 pad=2）
4. **第 312 行** `padded = np.pad(traj, ((pad, pad), (0, 0)), mode="edge")` — 对时间轴（第0维）两端用边缘值填充，使得开头和结尾的帧也可以计算窗口平均值
   - `mode="edge"` 表示用边界值重复填充（而不是补零）
5. **第 313–315 行** 对每一帧，取以该帧为中心的窗口 `[i : i+window]`，计算所有通道的平均值
6. **返回** 平滑后的轨迹，形状与输入相同

**举例**：`window=3` 时，第 i 帧的输出 = (第 i-1 帧 + 第 i 帧 + 第 i+1 帧) / 3

---

### 第三部分：打印配置信息（第 584–589 行）

```python
print(f"CSV: {csv_path}")
print(f"Selected frames: {dof29.shape[0]}")
print(f"Mode: {mode}, quat_format={quat_format}")
print(f"Base enabled: {enable_base}, arm_sdk upper-body joints enabled: {enable_joints}")
print(f"Waist disabled (locked to live position): {disable_waist}")
print(f"Upper-body joints sent (17): {upper_body_names()}")
```

**作用**：打印关键配置摘要，帮助操作者确认参数正确。

**`upper_body_names()` 详解（第 122–123 行）**：

```python
def upper_body_names() -> list[str]:
    return [G1_JOINT_NAMES_29[int(i)] for i in UPPER_BODY_INDICES]
```

- 遍历 `UPPER_BODY_INDICES`（第 116–119 行定义的 17 个索引），从 `G1_JOINT_NAMES_29`（第 78–108 行）中取出对应的关节名称
- `UPPER_BODY_INDICES` = `[15,16,17,18,19,20,21, 22,23,24,25,26,27,28, 12,13,14]`
  - 索引 15–21：左臂 7 关节（左肩俯仰/横滚/偏航、左肘、左腕横滚/俯仰/偏航）
  - 索引 22–28：右臂 7 关节（同上，右侧）
  - 索引 12–14：腰部 3 关节（腰部偏航/横滚/俯仰）

---

### 第四部分：连接机器人（第 591–600 行）

```python
needs_robot = execute or use_live_state
```

**作用**：判断是否需要连接机器人。只要需要实际执行（`execute=True`）或需要读取当前关节状态（`use_live_state=True`），就必须建立 DDS 连接。

```python
robot: DirectUnitreeG1ArmSdk | None = None
if needs_robot:
    robot = DirectUnitreeG1ArmSdk(network_interface)
    robot.connect(need_arm_sdk=enable_joints, need_loco=enable_base)
    live_q29 = robot.wait_low_state(lowstate_timeout)
    print(f"First 6 live q: {live_q29[:6].round(4).tolist()}")
else:
    live_q29 = dof29[0].copy()
    print("No live state requested. Dry-run uses CSV first frame as joint reference.")
```

**逐行分析**：

1. `robot = DirectUnitreeG1ArmSdk(network_interface)` — 创建机器人控制对象

2. `robot.connect(need_arm_sdk=enable_joints, need_loco=enable_base)` — 建立 DDS 连接。

---

#### `DirectUnitreeG1ArmSdk` 类详解（第 126–245 行）

这是封装宇树 G1 机器人高层控制接口的类。

**`__post_init__` 方法（第 130–136 行）**：

```python
def __post_init__(self) -> None:
    self.low_state: Any | None = None       # 机器人底层状态（关节角度等）
    self.arm_cmd: Any | None = None         # 上肢控制指令消息
    self.arm_pub: Any | None = None         # 上肢控制指令发布器
    self.loco: Any | None = None            # 底盘运动客户端
    self.crc: Any | None = None             # CRC 校验计算器
    self._handles: dict[str, Any] = {}      # 保存 DDS 句柄的字典
```

由于使用了 `@dataclass` 装饰器，`__post_init__` 在 `__init__` 之后自动调用，用于初始化非字段属性。

---

**`connect()` 方法（第 138–165 行）**：

```python
def connect(self, need_arm_sdk: bool, need_loco: bool) -> None:
```

1. **第 139–147 行** — 导入宇树 SDK 模块：
   - `ChannelFactoryInitialize`：**宇树 SDK 函数**，初始化 DDS（Data Distribution Service）通信工厂，参数 `0` 表示使用默认域 ID，`network_interface` 指定使用哪个网卡进行通信
   - `ChannelPublisher`：**宇树 SDK 类**，用于向 DDS 话题发布消息
   - `ChannelSubscriber`：**宇树 SDK 类**，用于订阅 DDS 话题消息
   - `LocoClient`：**宇树 SDK 类**，G1 机器人底盘运动客户端，封装了与底盘行走控制器的通信
   - `unitree_hg_msg_dds__LowCmd_`：**宇树 SDK 函数**，创建一个空的底层控制指令消息对象
   - `LowCmd_`：**宇树 SDK DDS 消息类型**，底层控制指令的 DDS 消息结构体
   - `LowState_`：**宇树 SDK DDS 消息类型**，底层状态反馈的 DDS 消息结构体
   - `CRC`：**宇树 SDK 工具类**，计算 DDS 消息的 CRC 校验和，用于保证消息完整性

2. **第 149–150 行**：
   ```python
   print(f"Initializing Unitree DDS on interface: {self.network_interface}")
   ChannelFactoryInitialize(0, self.network_interface)
   ```
   - `ChannelFactoryInitialize(0, self.network_interface)`：初始化 DDS 通道工厂。第一个参数 `0` 是 DDS 域 ID（同域内的节点才能互相发现和通信）；第二个参数指定网络接口名称（如 `"eth0"`）。这个调用必须在所有其他 DDS 操作之前完成。

3. **第 151 行** `self.crc = CRC()` — 创建 CRC 校验器实例

4. **第 153–155 行** 订阅机器人状态：
   ```python
   lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
   lowstate_sub.Init(self._low_state_handler, 10)
   ```
   - `ChannelSubscriber("rt/lowstate", LowState_)`：创建一个订阅者，监听 DDS 话题 `"rt/lowstate"`（机器人底层状态话题），消息类型为 `LowState_`
   - `lowstate_sub.Init(self._low_state_handler, 10)`：初始化订阅者，注册回调函数 `_low_state_handler`，队列深度为 10（缓存最多 10 条消息）
   - G1 机器人会以固定频率（通常 500Hz–1000Hz）向 `rt/lowstate` 话题发布包含所有 29 个关节角度、角速度、力矩等信息的状态消息

5. **第 157–160 行** 创建上肢控制发布器（条件性）：
   ```python
   if need_arm_sdk:
       self.arm_cmd = unitree_hg_msg_dds__LowCmd_()
       self.arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
       self.arm_pub.Init()
   ```
   - `unitree_hg_msg_dds__LowCmd_()`：创建一个空的 `LowCmd_` 消息实例，后续填充关节指令数据
   - `ChannelPublisher("rt/arm_sdk", LowCmd_)`：创建一个发布者，将向 DDS 话题 `"rt/arm_sdk"` 发布 `LowCmd_` 类型的消息
   - `self.arm_pub.Init()`：初始化发布者，使其可以开始发送消息
   - `"rt/arm_sdk"` 是宇树 G1 的高层上肢控制通道：机器人端订阅该话题，读取其中的关节目标角度（q）、目标角速度（dq）、力矩前馈（tau）、PD 增益（kp, kd），然后通过内置的 PD 控制器驱动机器人上肢关节

6. **第 162–165 行** 创建底盘运动客户端（条件性）：
   ```python
   if need_loco:
       self.loco = LocoClient()
       self.loco.SetTimeout(10.0)
       self.loco.Init()
   ```
   - `LocoClient()`：创建底盘运动客户端，这是一个高层接口，用于控制 G1 的行走/站立/速度等
   - `self.loco.SetTimeout(10.0)`：设置请求超时时间为 10 秒
   - `self.loco.Init()`：初始化客户端，建立与底盘控制器的通信连接

---

**`_low_state_handler()` 回调方法（第 167–168 行）**：

```python
def _low_state_handler(self, msg: Any) -> None:
    self.low_state = msg
```

**作用**：这是 DDS 订阅者的回调函数。每当机器人通过 `rt/lowstate` 话题发布新的状态消息时，DDS 中间件会自动调用此函数，将最新状态消息存储到 `self.low_state` 中。后续通过 `get_current_q29_or_none()` 读取其中的关节角度数据。

---

**`wait_low_state()` 方法（第 170–177 行）**：

```python
def wait_low_state(self, timeout_s: float) -> np.ndarray:
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        q = self.get_current_q29_or_none()
        if q is not None:
            return q
        time.sleep(0.02)
    raise TimeoutError("Timed out waiting for rt/lowstate")
```

**作用**：阻塞等待直到收到第一个机器人状态消息。每 20ms 检查一次 `self.low_state` 是否已被回调填充。如果超时（默认 5 秒）仍未收到，抛出 `TimeoutError`。这是必要的同步步骤，确保后续轨迹规划使用的"当前关节角度"是有效的。

---

**`get_current_q29_or_none()` 方法（第 179–186 行）**：

```python
def get_current_q29_or_none(self) -> np.ndarray | None:
    st = self.low_state
    if st is None:
        return None
    try:
        return np.asarray([st.motor_state[i].q for i in range(G1_NUM_MOTOR)], dtype=float)
    except Exception:
        return None
```

**作用**：从 `self.low_state`（`LowState_` 消息）中提取 29 个电机的当前角度 `q`。
- `st.motor_state` 是一个包含 29+ 个电机状态结构的列表，每个元素有 `q`（角度）、`dq`（角速度）、`tau`（力矩）等字段
- `G1_NUM_MOTOR = 29`，定义在第 75 行
- 如果 `low_state` 为 `None` 或解析失败，返回 `None`

---

**`send_base_velocity()` 方法（第 188–193 行）**：

```python
def send_base_velocity(self, vx: float, vy: float, wz: float, duration_s: float) -> None:
    if self.loco is None:
        raise RuntimeError("LocoClient is not initialized")
    code = self.loco.SetVelocity(float(vx), float(vy), float(wz), float(duration_s))
    if code != 0:
        raise RuntimeError(f"SetVelocity failed with code={code}")
```

**作用**：向底盘发送速度指令。
- `self.loco.SetVelocity(vx, vy, wz, duration_s)`：**宇树 SDK LocoClient 方法**，设置底盘在自身坐标系下的运动速度：
  - `vx`：前进方向线速度（m/s）
  - `vy`：侧向线速度（m/s）
  - `wz`：绕 Z 轴（垂直轴）旋转的角速度（rad/s）
  - `duration_s`：该速度指令持续的时间（秒）。机器人会在此时间内按指定速度运动，之后自动停止（这是一种安全机制，防止指令丢失导致机器人持续运动）
- 返回 `code=0` 表示成功；非零表示失败

---

**`stop_base()` 方法（第 195–203 行）**：

```python
def stop_base(self) -> None:
    if self.loco is None:
        return
    for _ in range(2):
        try:
            self.send_base_velocity(0.0, 0.0, 0.0, 0.1)
        except Exception as exc:
            print(f"Warning: stop base failed: {exc}")
        time.sleep(0.02)
```

**作用**：紧急停止底盘运动。发送两次零速度指令（每次持续 0.1 秒）以确保停止。发送两次是一种冗余安全措施——即使第一次丢失或失败，第二次也能生效。

---

**`send_arm_sdk_q()` 方法（第 205–230 行）**：

```python
def send_arm_sdk_q(
    self,
    q29: np.ndarray,      # 29个关节的目标角度
    dq29: np.ndarray,      # 29个关节的目标角速度
    kp: float,             # PD 比例增益
    kd: float,             # PD 微分增益
    weight: float,         # arm_sdk 启用权重 [0, 1]
) -> None:
```

1. **第 213–214 行** 检查 `arm_cmd`、`arm_pub`、`crc` 是否已初始化

2. **第 215–216 行** 将输入转换为 NumPy 数组并变形为 `(29,)` 形状：
   ```python
   q29 = np.asarray(q29, dtype=float).reshape(G1_NUM_MOTOR)
   dq29 = np.asarray(dq29, dtype=float).reshape(G1_NUM_MOTOR)
   ```

3. **第 218 行** 设置 arm_sdk 启用标志：
   ```python
   self.arm_cmd.motor_cmd[ARM_SDK_ENABLE_INDEX].q = float(np.clip(weight, 0.0, 1.0))
   ```
   - `ARM_SDK_ENABLE_INDEX = 29`（第 76 行）：G1 机器人的 `motor_cmd` 数组第 30 个元素（索引 29）是一个特殊标志位
   - 将 `motor_cmd[29].q` 设置为 0.0 到 1.0 之间的值：`0.0` = 禁用 arm_sdk 控制，`1.0` = 完全启用。逐步增加此值可以平滑接管上肢控制权，避免突然切换导致的抖动
   - `np.clip(weight, 0.0, 1.0)` 确保值在合法范围内

4. **第 219–225 行** 填充上肢关节指令：
   ```python
   for joint in UPPER_BODY_INDICES.tolist():
       mc = self.arm_cmd.motor_cmd[int(joint)]
       mc.tau = 0.0              # 力矩前馈（设为0，仅靠PD控制）
       mc.q = float(q29[int(joint)])    # 目标关节角度
       mc.dq = float(dq29[int(joint)])  # 目标关节角速度（前馈项）
       mc.kp = float(kp)         # 比例增益
       mc.kd = float(kd)         # 微分增益
   ```
   - 仅填充 17 个上肢关节（`UPPER_BODY_INDICES`），不涉及 12 个腿部关节
   - 每个关节的 `motor_cmd` 结构包含：
     - `tau`：前馈力矩（Nm），这里设为 0，完全依赖 PD 控制
     - `q`：目标关节角度（rad）
     - `dq`：目标关节角速度（rad/s）
     - `kp`：位置误差的比例增益
     - `kd`：速度误差的微分增益
   - 机器人端的控制器会执行：`tau_command = kp * (q_target - q_actual) + kd * (dq_target - dq_actual) + tau_feedforward`

5. **第 227–230 行** 计算 CRC 并发布消息：
   ```python
   self.arm_cmd.crc = self.crc.Crc(self.arm_cmd)
   ok = self.arm_pub.Write(self.arm_cmd)
   if not ok:
       raise RuntimeError("arm_sdk publish failed")
   ```
   - `self.crc.Crc(self.arm_cmd)`：**宇树 SDK CRC 方法**，计算整个 `LowCmd_` 消息的 CRC 校验和，并赋值给消息的 `crc` 字段。机器人端会验证 CRC，不匹配的消息会被丢弃，这是一种安全机制
   - `self.arm_pub.Write(self.arm_cmd)`：**宇树 SDK Publisher 方法**，将填充好的 `LowCmd_` 消息通过 DDS 发布到 `"rt/arm_sdk"` 话题。返回 `True` 表示发布成功

---

**`release_arm_sdk()` 方法（第 232–244 行）**：

```python
def release_arm_sdk(self, steps: int = 25, period: float = 0.02) -> None:
```

**作用**：平滑释放 arm_sdk 控制权。通过逐步降低 `motor_cmd[29].q`（启用权重）从 1.0 到 0.0，避免突然断开导致的抖动。

1. **第 235 行** `for i in range(max(1, int(steps))):` — 默认 25 步，最少 1 步
2. **第 236 行** `weight = 1.0 - float(i + 1) / float(max(1, int(steps)))` — 线性递减：第一步 weight=0.96，最后一步 weight=0.0
3. **第 237 行** 将递减后的权重写入 `motor_cmd[29].q`
4. **第 238–242 行** 计算 CRC 并发布，如果失败则打印警告并终止释放循环
5. **第 244 行** `time.sleep(max(0.0, float(period)))` — 每步之间等待 `period` 秒（默认 0.02s），总计约 0.5 秒完成释放

---

继续 `run_gmr_direct_arm_sdk` 的第 600 行：

```python
else:
    live_q29 = dof29[0].copy()
    print("No live state requested. Dry-run uses CSV first frame as joint reference.")
```

**作用**：如果不连接机器人（演练模式），则用 CSV 第一帧的关节角度作为"当前姿态"的参考值。这样即使是演练模式，轨迹计算也有意义的参考值。

---

### 第五部分：构建运动轨迹（第 602–624 行）

```python
joint_traj = build_joint_trajectory(
    dof29,
    live_q29,
    mode=mode,
    scale=scale,
    max_joint_delta=max_joint_delta,
    blend_frames=blend_frames,
    disable_waist=disable_waist,
)
```

**作用**：调用 `build_joint_trajectory()` 将原始 CSV 关节数据转换为实际可发送给机器人的关节轨迹。

**`build_joint_trajectory()` 详解（第 319–349 行）**：

```python
def build_joint_trajectory(
    csv_q29: np.ndarray,        # CSV中的关节角度 (T, 29)
    live_q29: np.ndarray,       # 当前机器人关节角度 (29,)
    mode: str,                  # "relative" 或 "absolute"
    scale: float,               # 相对模式缩放因子
    max_joint_delta: float,     # 最大关节偏移限制
    blend_frames: int,          # 混合过渡帧数
    disable_waist: bool = False,# 是否锁定腰部
) -> np.ndarray:
```

1. **第 328 行** `live = np.asarray(live_q29, dtype=float).reshape(-1)` — 将当前关节角度展平为一维数组

2. **第 329–330 行** 验证 CSV 数据列数与 live_q29 长度一致（都是 29）

3. **第 332 行** 核心逻辑——根据模式计算目标轨迹：
   ```python
   cmd = csv_q29.copy() if mode == "absolute" else live[None, :] + (csv_q29 - csv_q29[0:1]) * scale
   ```
   - **绝对模式** (`"absolute"`)：直接使用 CSV 中的关节角度作为目标值（假设机器人初始姿态与 CSV 第一帧一致）
   - **相对模式** (`"relative"`)：
     - `csv_q29 - csv_q29[0:1]`：提取 CSV 中每一帧相对于第一帧的关节偏移量，形状 `(T, 29)`
     - `* scale`：按缩放因子调整偏移幅度（`scale=1.0` 表示 1:1 还原，`scale=0.5` 表示只做一半幅度）
     - `live[None, :] + ...`：将偏移量叠加到当前机器人关节角度上
     - `live[None, :]` 通过增加维度将 `(29,)` 变为 `(1, 29)`，与 `(T, 29)` 广播相加

4. **第 333–334 行** 安全限幅：
   ```python
   if max_joint_delta > 0.0:
       cmd = np.clip(cmd, live[None, :] - max_joint_delta, live[None, :] + max_joint_delta)
   ```
   - 将每个关节的目标角度限制在「当前角度 ± max_joint_delta」范围内
   - 这是关键的安全机制：防止因 CSV 数据异常或参考姿态差异过大导致的剧烈运动

5. **第 339–341 行** 腰部锁定（条件性）：
   ```python
   if disable_waist:
       for j in WAIST_INDICES:
           cmd[:, j] = live[j]
   ```
   - `WAIST_INDICES = (12, 13, 14)`（第 115 行）对应 waist_yaw、waist_roll、waist_pitch
   - 将这三个关节的全部轨迹帧设为当前角度，即腰部完全不运动
   - 这是为了避免与 G1 内置的 Loco 平衡控制器产生冲突

6. **第 343–348 行** 平滑混合过渡：
   ```python
   blend_frames = max(0, min(int(blend_frames), cmd.shape[0]))
   if blend_frames > 0:
       target = cmd[:blend_frames].copy()
       for i in range(blend_frames):
           alpha = (i + 1) / float(blend_frames)
           cmd[i] = live * (1.0 - alpha) + target[i] * alpha
   ```
   - 在前 `blend_frames` 帧中，对每个关节执行线性插值（LERP）：`cmd = live * (1-α) + target * α`
   - `α` 从 `1/blend_frames` 线性增长到 `1.0`
   - 效果：关节从当前姿态平滑过渡到目标轨迹的第一帧，避免启动时的阶跃跳变
   - 例如 `blend_frames=25`、`control_hz=50` 时，过渡过程持续 0.5 秒

---

```python
base_vel = build_base_velocity(
    root_pos, root_rot, fps=fps, speed=speed,
    quat_format=quat_format, scale_xy=base_scale_xy, scale_yaw=base_scale_yaw,
    max_vx=max_vx, max_vy=max_vy, max_wz=max_wz,
)
```

**作用**：调用 `build_base_velocity()` 从根节点位姿计算底盘速度指令。

**`build_base_velocity()` 详解（第 362–396 行）**：

```python
def build_base_velocity(
    root_pos: np.ndarray,      # 根节点位置 (T, 3)
    root_rot: np.ndarray,      # 根节点四元数 (T, 4)
    fps: float,                # CSV 帧率
    speed: float,              # 回放速度倍率
    quat_format: str,          # 四元数格式
    scale_xy: float,           # XY 速度缩放
    scale_yaw: float,          # 偏航速度缩放
    max_vx: float,             # X 速度限制
    max_vy: float,             # Y 速度限制
    max_wz: float,             # 偏航角速度限制
) -> np.ndarray:
```

1. **第 374 行** `pos = np.asarray(root_pos, dtype=float)` — 确保为浮点数组

2. **第 375 行** `yaw = quat_to_yaw(root_rot, quat_format)` — 调用 `quat_to_yaw()` 从四元数提取偏航角（Yaw）

   **`quat_to_yaw()` 详解（第 272–283 行）**：
   ```python
   def quat_to_yaw(quat: np.ndarray, fmt: str) -> np.ndarray:
   ```
   - 验证输入形状为 `(T, 4)`
   - 归一化四元数（防止数值误差导致的非单位四元数）
   - 根据 `fmt`（`"xyzw"` 或 `"wxyz"`）解析 w, x, y, z 分量
   - 使用公式计算偏航角：`yaw = arctan2(2(wz + xy), 1 - 2(y² + z²))`
   - `np.unwrap()` 对角度序列进行解缠处理：当角度跳变超过 π 时自动修正（如 π → -π 变为 π → π+δ），保证角度连续性
   - 这是将单位四元数转换为绕 Z 轴旋转角度的标准公式

3. **第 376 行** `dt = 1.0 / max(1.0, fps * max(0.01, speed))` — 计算实际的时间步长：
   - `fps * speed` 是回放的有效帧率（如 fps=50, speed=2.0 → 100Hz 等效）
   - 确保 fps*speed 不小于 0.01，dt 不小于 0

4. **第 378–384 行** 有限差分计算速度：
   ```python
   dpos = np.zeros((pos.shape[0], 3), dtype=float)     # 位置差分（世界坐标系速度）
   dyaw = np.zeros(pos.shape[0], dtype=float)           # 偏航角差分
   if pos.shape[0] > 1:
       dpos[:-1] = pos[1:] - pos[:-1]   # 前向差分
       dpos[-1] = dpos[-2]              # 最后一帧复制前一帧
       dyaw[:-1] = yaw[1:] - yaw[:-1]
       dyaw[-1] = dyaw[-2]
   ```
   - 使用前向差分近似导数：`v[i] ≈ (pos[i+1] - pos[i]) / dt`
   - 最后一帧没有下一帧，直接复用前一帧的速度

5. **第 386–391 行** 将世界坐标系速度转换为机器人自身坐标系：
   ```python
   vel_world = dpos[:, :2] / dt * scale_xy   # 世界坐标系 XY 速度，乘缩放因子
   c = np.cos(yaw)
   s = np.sin(yaw)
   vx = c * vel_world[:, 0] + s * vel_world[:, 1]    # 旋转到机器人坐标系
   vy = -s * vel_world[:, 0] + c * vel_world[:, 1]
   wz = dyaw / dt * scale_yaw
   ```
   - `vel_world` 是世界坐标系下的速度矢量（CSV 数据通常在世界坐标系）
   - 使用当前帧的偏航角 `yaw` 构建旋转矩阵，将世界速度转换到机器人自身坐标系
   - 数学上：`[vx; vy] = R(-yaw) * vel_world`，其中 R 是 2D 旋转矩阵
   - G1 的 `SetVelocity(vx, vy, wz)` 使用的是机器人自身坐标系：`vx` = 前进方向，`vy` = 左侧方向

6. **第 392–395 行** 组合并限幅：
   ```python
   cmd = np.stack([vx, vy, wz], axis=1)
   cmd[:, 0] = np.clip(cmd[:, 0], -abs(max_vx), abs(max_vx))
   cmd[:, 1] = np.clip(cmd[:, 1], -abs(max_vy), abs(max_vy))
   cmd[:, 2] = np.clip(cmd[:, 2], -abs(max_wz), abs(max_wz))
   ```
   - 将 vx, vy, wz 堆叠为 `(T, 3)` 数组
   - 分别对三个通道进行限幅

**返回**：形状为 `(T, 3)` 的数组，每行 `[vx, vy, wz]`

---

```python
period = 1.0 / max(1.0, control_hz * max(0.01, speed))
```

**作用**：计算控制循环的实际周期（秒/帧）。
- `control_hz * speed`：有效控制频率。如 control_hz=50, speed=1.0 → 50Hz → period=0.02s
- 确保分母不小于 1.0，即周期不大于 1 秒

---

```python
joint_dq = build_joint_velocity(joint_traj, period, max_dq)
```

**作用**：调用 `build_joint_velocity()` 从关节轨迹计算关节目标角速度（速度前馈）。

**`build_joint_velocity()` 详解（第 352–359 行）**：

```python
def build_joint_velocity(joint_traj: np.ndarray, period: float, max_dq: float) -> np.ndarray:
```

1. **第 353 行** `dq = np.zeros_like(joint_traj)` — 初始化全零数组
2. **第 354–356 行** 前向有限差分：
   ```python
   if joint_traj.shape[0] > 1:
       dq[:-1] = (joint_traj[1:] - joint_traj[:-1]) / period
       dq[-1] = dq[-2]
   ```
   - `dq[i] = (q[i+1] - q[i]) / period` — 用位置差分近似速度
   - 最后一帧复制前一帧的速度
3. **第 357–358 行** 速度限幅：`dq = np.clip(dq, -abs(max_dq), abs(max_dq))`

**作用**：速度前馈项（`dq` 字段）让机器人的 PD 控制器提前知道目标速度，可以改善跟踪性能，减少跟踪延迟。

---

### 第六部分：打印摘要信息（第 626–630 行）

```python
print_summary(root_pos, joint_traj, base_vel, live_q29, dof29, disable_waist=disable_waist)
duration = joint_traj.shape[0] * period
print(f"\nEstimated playback duration: {duration:.2f}s")
print(f"Command period: {period:.4f}s, control_hz={control_hz}")
print(f"arm_sdk kp={arm_kp}, kd={arm_kd}")
```

**`print_summary()` 详解（第 399–429 行）**：

打印以下信息：
- 总帧数
- 根节点 XYZ 位置的最小/最大值
- 底盘速度（vx, vy, wz）的最小/最大值
- 如果腰部被锁定，打印提示信息
- 17 个上肢关节的详细对比表，每个关节列出：当前角度 (live/ref)、CSV 原始角度范围、实际指令角度范围。这使操作者可以在执行前目视检查轨迹是否合理

---

### 第七部分：演练模式判断（第 632–634 行）

```python
if not execute:
    print("Dry-run only. Add --execute or pass execute=True to command the real robot.")
    return 0
```

**作用**：如果 `execute=False`（默认值），则在此处退出。所有前面的轨迹计算和摘要打印都已执行，但不会向机器人发送任何指令。这就是"演练"（dry-run）模式。

---

### 第八部分：安全检查（第 638–646 行）

```python
print("\nSafety checklist:")
print("  [ ] This script bypasses TongRobot and commands Unitree SDK2 directly")
print("  [ ] Joint command path is rt/arm_sdk, not rt/lowcmd")
print("  [ ] Only upper-body 17 joints are commanded; leg joints from CSV are ignored")
print("  [ ] Base velocity and upper-body motion have been tested separately first")
print("  [ ] Operator can trigger hardware E-stop immediately")
if not yes and not ask_yes_no("Execute direct Unitree high-level commands now?"):
    print("Aborted.")
    return 1
```

**作用**：在执行前展示安全检查清单并要求操作者确认。

**`ask_yes_no()` 详解（第 253–255 行）**：

```python
def ask_yes_no(prompt: str) -> bool:
    ans = input(f"{prompt} [y/N]: ").strip().lower()
    return ans in {"y", "yes"}
```

- 显示提示并等待用户输入，默认选项为 N（否）
- 只有输入 `"y"` 或 `"yes"`（不区分大小写）时才返回 `True`
- 如果用户不确认或 `--yes` 标志被设置，则函数返回 1（中止）

---

### 第九部分：实时回放循环（第 648–669 行）

```python
try:
    sent = replay(
        robot, joint_traj, joint_dq, base_vel,
        control_hz=control_hz, speed=speed,
        enable_base=enable_base, enable_joints=enable_joints,
        arm_kp=arm_kp, arm_kd=arm_kd,
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
```

**`replay()` 函数详解（第 432–462 行）**：

```python
def replay(
    robot: DirectUnitreeG1ArmSdk,
    joint_traj: np.ndarray,     # (T, 29) 关节角度轨迹
    joint_dq: np.ndarray,       # (T, 29) 关节角速度
    base_vel: np.ndarray,       # (T, 3) 底盘速度
    control_hz: float,          # 控制频率
    speed: float,               # 速度倍率
    enable_base: bool,          # 是否发送底盘指令
    enable_joints: bool,        # 是否发送关节指令
    arm_kp: float,              # PD 比例增益
    arm_kd: float,              # PD 微分增益
) -> int:
```

1. **第 444 行** `period = 1.0 / max(1.0, control_hz * max(0.01, speed))` — 计算实际控制周期

2. **第 445 行** `next_t = time.monotonic()` — 记录起始时间点，用于后续的节拍控制

3. **第 446 行** `sent = 0` — 已发送帧数计数器

4. **第 447 行** `for frame_idx, (q, dq, vel) in enumerate(zip(joint_traj, joint_dq, base_vel)):` — 逐帧迭代，同时取出关节角度 `q`、关节角速度 `dq`、底盘速度 `vel`

5. **第 448–449 行** 发送底盘速度（条件性）：
   ```python
   if enable_base:
       robot.send_base_velocity(float(vel[0]), float(vel[1]), float(vel[2]), period * 2.0)
   ```
   - 发送三个速度分量 `(vx, vy, wz)`
   - `duration_s=period * 2.0`：持续时间设为两倍控制周期。这是一种安全设计——如果下一帧指令因任何原因丢失或延迟，底盘不会立即停止（有缓冲时间）；但如果持续丢失，两次控制周期后速度会自动归零

6. **第 450–452 行** 发送关节指令（条件性）：
   ```python
   if enable_joints:
       weight = min(1.0, float(frame_idx + 1) / 25.0)
       robot.send_arm_sdk_q(q, dq, kp=arm_kp, kd=arm_kd, weight=weight)
   ```
   - `weight` 是 arm_sdk 启用权重，前 25 帧从 `1/25=0.04` 线性增长到 `1.0`
   - 这使机器人平滑地从无控制（完全由自身控制器维持）过渡到完全接受 arm_sdk 指令
   - 帧索引 ≥ 25 后，`weight = 1.0`（完全启用）

7. **第 453 行** `sent += 1` — 递增计数器

8. **第 454–459 行** 节拍控制（定时循环）：
   ```python
   next_t += period
   sleep_s = next_t - time.monotonic()
   if sleep_s > 0:
       time.sleep(sleep_s)
   else:
       next_t = time.monotonic()
   ```
   - `next_t += period`：计算下一帧的理想发送时刻
   - `sleep_s = next_t - time.monotonic()`：计算需要等待的时间
   - 如果 `sleep_s > 0`：休眠到下一帧时刻
   - 如果 `sleep_s <= 0`：说明本帧处理耗时超过了周期（掉帧），重置 `next_t` 为当前时间以避免追赶导致的指令突发
   - 使用 `time.monotonic()` 而非 `time.time()`，因为它不受系统时间调整的影响，保证计时单调递增

9. **第 460–461 行** 循环结束后停止底盘：
   ```python
   if enable_base:
       robot.stop_base()
   ```

10. **第 462 行** `return sent` — 返回已发送的总帧数

---

**`try/finally` 块的作用**：

```python
try:
    sent = replay(...)
    ...
except KeyboardInterrupt:
    print("\nInterrupted by operator.")
    return 130
finally:
    if robot is not None:
        robot.stop_base()
        robot.release_arm_sdk()
```

- `KeyboardInterrupt`（Ctrl+C）被捕获，打印提示后返回退出码 130（Unix 惯例，128 + SIGINT=2）
- **`finally` 块至关重要**：无论正常结束、异常退出还是被用户中断，都会执行：
  - `robot.stop_base()`：停止底盘运动
  - `robot.release_arm_sdk()`：平滑释放 arm_sdk 控制权（将 weight 从 1.0 降到 0.0）
- 这确保了机器人不会在脚本异常退出后仍然保持最后一帧的运动状态

---

## 数据流总览

```
CSV文件 (36列)
  │
  ├─ load_gmr_csv() ────────► root_pos (T,3) + root_rot (T,4) + dof29 (T,29)
  │
  ├─ select_frames() ───────► 按起止帧/步长切片
  │
  ├─ moving_average() ──────► 对 dof29 做滑动平均平滑
  │
  ├─ 机器人连接 ─────────────► live_q29 (当前29关节角度, 来自 rt/lowstate)
  │
  ├─ build_joint_trajectory()► joint_traj (T,29) 目标关节角度（相对/绝对/限幅/混合）
  │
  ├─ build_base_velocity() ──► base_vel (T,3) [vx, vy, wz]（世界→机器人坐标转换）
  │
  ├─ build_joint_velocity() ─► joint_dq (T,29) 目标关节角速度（有限差分）
  │
  ├─ print_summary() ────────► 打印轨迹摘要供人工检查
  │
  └─ replay() ──────────────► 逐帧发送:
                                ├─ LocoClient.SetVelocity(vx,vy,wz) → 底盘运动
                                └─ ChannelPublisher.Write(arm_cmd) → rt/arm_sdk → 上肢关节PD控制
```

---

## 使用的宇树 SDK 接口汇总

| SDK 接口 | 类型 | 作用 |
|----------|------|------|
| `ChannelFactoryInitialize(0, iface)` | 函数 | 初始化 DDS 通信工厂，绑定指定网卡 |
| `ChannelSubscriber("rt/lowstate", LowState_)` | 类 | 订阅机器人底层状态话题 |
| `ChannelSubscriber.Init(handler, queue_depth)` | 方法 | 注册回调并设置消息队列深度 |
| `ChannelPublisher("rt/arm_sdk", LowCmd_)` | 类 | 创建上肢控制指令发布器 |
| `ChannelPublisher.Init()` | 方法 | 初始化发布器 |
| `ChannelPublisher.Write(msg)` | 方法 | 通过 DDS 发布一条消息 |
| `unitree_hg_msg_dds__LowCmd_()` | 函数 | 创建一个空的 LowCmd 消息实例 |
| `LowCmd_` | DDS 类型 | 底层控制指令消息结构体（含 motor_cmd 数组） |
| `LowState_` | DDS 类型 | 底层状态消息结构体（含 motor_state 数组） |
| `LocoClient()` | 类 | G1 底盘运动控制客户端 |
| `LocoClient.SetTimeout(s)` | 方法 | 设置请求超时 |
| `LocoClient.Init()` | 方法 | 初始化底盘客户端 |
| `LocoClient.SetVelocity(vx,vy,wz,dur)` | 方法 | 发送底盘速度指令（机器人自身坐标系） |
| `CRC()` | 类 | CRC 校验计算器 |
| `CRC.Crc(msg)` | 方法 | 计算消息的 CRC 校验和 |

---

## 关键安全机制

1. **演练模式（dry-run）**：`execute=False` 时只计算轨迹、打印摘要，不发送任何指令
2. **关节偏移限制（max_joint_delta）**：防止关节角度突变，默认 ±0.5 rad（约 28.6°）
3. **关节速度限制（max_dq）**：防止关节转速过快，默认 ±4.0 rad/s
4. **底盘速度限制（max_vx/vy/wz）**：分别限制各方向速度
5. **平滑混合（blend_frames）**：启动时从当前姿态线性过渡到目标轨迹
6. **arm_sdk 权重渐变**：启用（前 25 帧从 0→1）和释放（25 帧从 1→0）都是逐步的
7. **CRC 校验**：每条 DDS 消息带 CRC，机器人端校验不通过则丢弃
8. **速度指令有限时长**：`SetVelocity` 的 `duration` 参数确保指令超时后自动停止
9. **finally 安全释放**：无论何种原因退出，都会停止底盘并释放 arm_sdk
10. **人工确认（--yes 跳过）**：执行前展示安全检查清单并要求确认
11. **腰部锁定选项（--disable-waist）**：避免与 G1 内置平衡控制器冲突
