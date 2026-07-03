# Unitree G1 SDK 真机部署接口详解

## 概述

宇树 G1 机器人的 Python SDK（`unitree_sdk2_python`）基于 **DDS（Data Distribution Service）** 中间件，使用 **Eclipse Cyclone DDS** 实现。SDK 提供两种上肢控制方式：

| 方式 | 通道 | 适用场景 | 安全性 |
|------|------|---------|--------|
| **arm_sdk**（推荐） | `rt/arm_sdk` | 仅控制上肢运动轨迹回放 | ✅ 高 — 不影响腿部平衡 |
| **lowcmd** | `rt/lowcmd` | 全关节底层控制 | ⚠️ 需要先获取控制权（MotionSwitcher） |

本文档基于 `replay_arms_on_g1.py` 脚本使用的接口展开。

---

## 一、SDK 整体架构

```
replay_arms_on_g1.py
       │
       ├── ChannelFactoryInitialize(0, "eth0")   ← 初始化 DDS 环境
       │
       ├── ChannelPublisher("rt/arm_sdk", LowCmd_)  ← 创建发布者
       │     └── arm_sdk_publisher.Init()            ← 初始化发布通道
       │     └── arm_sdk_publisher.Write(low_cmd)    ← 循环发送指令
       │
       ├── CRC().Crc(low_cmd)                        ← 计算校验和
       │
       └── RecurrentThread(interval=0.02, ...)       ← 50Hz 控制循环
```

---

## 二、DDS 通信层

### 2.1 ChannelFactoryInitialize — 初始化 DDS 环境

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

ChannelFactoryInitialize(0, "eth0")      # 指定网络接口
ChannelFactoryInitialize(0)               # 自动检测网络接口
```

**参数说明**：
| 参数 | 类型 | 说明 |
|------|------|------|
| `id` | `int` | DDS Domain ID，固定为 `0` |
| `networkInterface` | `str` 或 `None` | 网络接口名（如 `"eth0"`, `"enp2s0"`），`None` 为自动检测 |

**内部实现**（`channel.py:298`）：
```python
def ChannelFactoryInitialize(id: int = 0, networkInterface: str = None):
    factory = ChannelFactory()
    if not factory.Init(id, networkInterface):
        raise Exception("channel factory init error.")
```

`ChannelFactory.Init()` 完成了三件事：
1. 根据 `networkInterface` 生成 Cyclone DDS XML 配置
2. 创建 DDS `Domain(id)` — DDS 通信域
3. 创建 `DomainParticipant(id)` — 域参与者（单例）

**G1 机器人默认网口**：机器人内置计算机通过以太网口与外界通信，通常为 `eth0`。如果不知道接口名，传 `None` 让 SDK 自动检测。

---

### 2.2 ChannelPublisher — 发布指令到机器人

```python
from unitree_sdk2py.core.channel import ChannelPublisher

arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
arm_sdk_publisher.Init()
```

**构造函数参数**：
| 参数 | 类型 | 说明 |
|------|------|------|
| `name` | `str` | DDS Topic 名称，G1 支持的 topic 见下表 |
| `type` | `IdlStruct` | 消息类型（DDS IDL 生成的数据类） |

**G1 支持的核心 Topic**：

| Topic 名 | 方向 | 消息类型 | 用途 |
|----------|------|---------|------|
| `rt/arm_sdk` | PC→机器人 | `LowCmd_` | 上肢 SDK 控制（仅 arm+waist 关节） |
| `rt/lowcmd` | PC→机器人 | `LowCmd_` | 全关节底层控制（需 MotionSwitcher） |
| `rt/lowstate` | 机器人→PC | `LowState_` | 机器人状态反馈（关节角度、IMU等） |

**关键方法**：

```python
# 初始化发布通道（创建 DDS DataWriter）
publisher.Init()

# 发送一条消息到机器人
# - sample: LowCmd_ 实例
# - timeout: 超时秒数，None 为无限等待
# 返回 True/False 表示是否发送成功
publisher.Write(sample, timeout=None)

# 关闭发布通道
publisher.Close()
```

**内部实现**（`channel.py:256`）：
```python
class ChannelPublisher:
    def __init__(self, name, type):
        factory = ChannelFactory()
        self.__channel = factory.CreateChannel(name, type)

    def Init(self):
        self.__channel.SetWriter(None)  # 创建 DDS DataWriter

    def Write(self, sample, timeout=None):
        return self.__channel.Write(sample, timeout)
```

`Write()` 内部会：
1. 等到至少有一个订阅者匹配（`publication_matched_count > 0`）
2. 调用 Cyclone DDS 的 `writer.write(sample)` 序列化并发送

---

### 2.3 ChannelSubscriber — 订阅机器人状态

```python
from unitree_sdk2py.core.channel import ChannelSubscriber

lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
lowstate_subscriber.Init(LowStateHandler, queueLen=10)
```

**参数说明**：
| 参数 | 类型 | 说明 |
|------|------|------|
| `handler` | `Callable` | 收到消息时的回调函数 |
| `queueLen` | `int` | 消息队列长度（>0 时使用独立线程分发） |

```python
def LowStateHandler(self, msg: LowState_):
    # msg.motor_state[i].q  → 第 i 个关节的当前角度 (rad)
    # msg.motor_state[i].dq → 第 i 个关节的当前速度 (rad/s)
    # msg.imu_state.rpy     → 机器人姿态 (roll, pitch, yaw)
    pass
```

---

## 三、消息类型

### 3.1 LowCmd_ — 控制指令消息

```python
@dataclass
class LowCmd_:
    mode_pr: uint8                          # 踝关节控制模式 (0=PR, 1=AB)
    mode_machine: uint8                     # 状态机模式
    motor_cmd: array[MotorCmd_, 35]         # 35 个电机的控制指令
    reserve: array[uint32, 4]               # 保留字段
    crc: uint32                             # CRC32 校验和
```

**`motor_cmd` 数组的 35 个槽位**：

| 索引范围 | 对应部位 | 说明 |
|---------|---------|------|
| 0–5 | 左腿 6 关节 | HipPitch/Roll/Yaw, Knee, AnklePitch/Roll |
| 6–11 | 右腿 6 关节 | 同上 |
| 12–14 | 腰部 3 关节 | Yaw, Roll, Pitch |
| 15–21 | 左臂 7 关节 | ShoulderPitch/Roll/Yaw, Elbow, WristRoll/Pitch/Yaw |
| 22–28 | 右臂 7 关节 | 同上 |
| 29 | 控制字 `kNotUsedJoint` | **arm_sdk 通道**：`q=1` 使能控制，`q=0` 释放 |
| 30–34 | 灵巧手/预留 | Dexterous hand |

---

### 3.2 MotorCmd_ — 单电机指令

```python
@dataclass
class MotorCmd_:
    mode: uint8         # 电机模式：1=使能(Enable), 0=失能(Disable)
    q: float32          # 目标关节角度 (rad)
    dq: float32         # 目标关节速度 (rad/s)，通常设为 0
    tau: float32        # 前馈力矩 (Nm)，通常设为 0
    kp: float32         # 位置增益（比例系数）
    kd: float32         # 速度增益（阻尼系数）
    reserve: uint32     # 保留字段
```

**构造一条电机指令**：
```python
cmd = MotorCmd_()
cmd.mode = 1            # 使能电机
cmd.q = 0.5             # 目标角度 0.5 rad（约 28.6°）
cmd.dq = 0.0            # 目标速度 0
cmd.tau = 0.0           # 无前馈力矩
cmd.kp = 40.0           # 位置增益
cmd.kd = 1.0            # 速度增益
```

**PD 控制原理**：机器人电机控制器内部运行一个 PD 控制器：
```
τ = kp × (q_target - q_current) + kd × (dq_target - dq_current) + tau_ff
```
- `kp` 越大，跟踪越快但可能震荡
- `kd` 越大，阻尼越强但响应变慢
- `tau` 提供额外的前馈力矩（补偿重力等）

**推荐的增益值**：

| 关节 | kp | kd | 说明 |
|------|-----|-----|------|
| 肩部 | 40 | 1.0 | 大关节，需要较高增益 |
| 肘部 | 40 | 1.0 | 同上 |
| 腕部 | 40 | 1.0 | 小关节，减小增益可降低抖动 |
| 腰部 | 60 | 1.5 | arm_sdk 模式下通常不需要设置 |

---

### 3.3 LowState_ — 机器人状态消息

```python
@dataclass
class LowState_:
    version: array[uint32, 2]               # 版本号
    mode_pr: uint8                          # 当前 PR/AB 模式
    mode_machine: uint8                     # 当前状态机模式
    tick: uint32                            # 时间戳（毫秒级递增计数器）
    imu_state: IMUState_                    # IMU 数据
    motor_state: array[MotorState_, 35]     # 35 个电机的反馈状态
    wireless_remote: array[uint8, 40]       # 遥控器数据
    reserve: array[uint32, 4]
    crc: uint32
```

**MotorState_ 中常用的字段**：
```python
motor_state[i].q       # 当前关节角度 (rad)
motor_state[i].dq      # 当前关节速度 (rad/s)
motor_state[i].tau_est # 估计力矩 (Nm)
motor_state[i].mode    # 当前电机模式
```

**IMUState_ 中常用的字段**：
```python
imu_state.rpy          # [roll, pitch, yaw] (rad)
imu_state.quaternion   # [w, x, y, z]
imu_state.gyroscope    # 角速度 [x, y, z] (rad/s)
imu_state.accelerometer # 加速度 [x, y, z] (m/s²)
```

---

## 四、CRC 校验

### 4.1 CRC().Crc() — 计算校验和

```python
from unitree_sdk2py.utils.crc import CRC

crc = CRC()
low_cmd.crc = crc.Crc(low_cmd)   # 必须在 Write 之前调用
```

**重要性**：机器人端会验证 CRC，校验和不匹配的指令会被丢弃。这是一个安全机制，防止传输错误导致意外运动。

**实现细节**（`crc.py`）：

```python
class CRC:
    def Crc(self, msg: IdlStruct):
        # 1. 识别消息类型（typename 字符串匹配）
        # 2. 将消息序列化为二进制（struct.pack，小端对齐）
        # 3. 调用 C 语言的 CRC32 校验函数（crc_aarch64.so / crc_amd64.so）
        return crc32_result
```

---

## 五、控制循环

### 5.1 RecurrentThread — 定时循环线程

```python
from unitree_sdk2py.utils.thread import RecurrentThread

thread = RecurrentThread(
    interval=0.02,          # 20ms = 50 Hz（与 GMR 数据帧率匹配）
    target=LowCmdWrite,     # 回调函数
    name="arm_replay"
)
thread.Start()              # 启动线程
```

**工作原理**（`thread.py`）：
```python
class RecurrentThread(Thread):
    def __LoopFunc(self):
        tfd = timerfd_create(1, 0)         # 创建 Linux timerfd
        spec = itimerspec.from_seconds(interval, interval)  # 设置周期
        timerfd_settime(tfd, 0, spec, None)

        while not self.__quit:
            self.__loopTarget(*args)        # 调用回调函数
            os.read(tfd, 8)                 # 阻塞等待下一个周期
```

使用 Linux 内核的 `timerfd` 机制实现精确的周期性唤醒，误差通常在微秒级。

**控制频率选择**：
| 频率 | `interval` | 适用场景 |
|------|-----------|---------|
| 50 Hz | 0.02 s | arm_sdk 回放（与 GMR 帧率匹配，无需插值） |
| 500 Hz | 0.002 s | lowcmd 全关节控制（需要插值） |

---

## 六、MotionSwitcher（lowcmd 模式专用）

如需通过 `rt/lowcmd` 通道控制全身关节，必须先获取控制权：

```python
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

msc = MotionSwitcherClient()
msc.SetTimeout(5.0)
msc.Init()

# 1. 检查当前模式
status, result = msc.CheckMode()
print(result['name'])  # 当前控制模式名称

# 2. 释放已有控制权
while result['name']:
    msc.ReleaseMode()
    status, result = msc.CheckMode()
    time.sleep(1)

# 3. 选择 "lowcmd" 模式
msc.SelectMode("lowcmd")
```

**RPC 客户端内部**（`client.py`）：
```python
class Client:
    def _RegistApi(self, apiId, priority):   # 注册 API
    def _Call(self, apiId, parameter):        # 同步调用（等待返回）
    def _CallNoReply(self, apiId, parameter): # 异步调用（不等待返回）
```

底层通过 DDS RPC（Request-Reply 模式）与机器人内置的服务进程通信。

---

## 七、arm_sdk 回放脚本中的完整调用流程

```python
# ========== Step 1: 初始化 DDS 环境 ==========
ChannelFactoryInitialize(0, "eth0")
# → 创建 Cyclone DDS Domain + Participant（单例）

# ========== Step 2: 创建发布者 ==========
arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
arm_sdk_publisher.Init()
# → 创建 Topic("rt/arm_sdk") → 创建 DataWriter → 等待机器人订阅者上线

# ========== Step 3: 创建 CRC 实例 ==========
crc = CRC()
# → 加载 crc_aarch64.so（Jetson Orin）或 crc_amd64.so（x86 PC）

# ========== Step 4: 构建 LowCmd_ 消息 ==========
low_cmd = unitree_hg_msg_dds__LowCmd_()  # 工厂函数，创建全零消息

# 逐帧填充手臂关节指令
for i, joint in enumerate(arm_joints):   # arm_joints = [15,16,...,28]
    low_cmd.motor_cmd[joint].mode = 0    # mode 设为 0（不覆盖默认模式）
    low_cmd.motor_cmd[joint].q = target  # 目标角度（从 GMR 数据读取）
    low_cmd.motor_cmd[joint].dq = 0.0
    low_cmd.motor_cmd[joint].kp = 40.0
    low_cmd.motor_cmd[joint].kd = 1.0
    low_cmd.motor_cmd[joint].tau = 0.0

low_cmd.motor_cmd[29].q = 1.0   # kNotUsedJoint = 1 → 使能 arm_sdk

# ========== Step 5: 计算 CRC ==========
low_cmd.crc = crc.Crc(low_cmd)

# ========== Step 6: 发送 ==========
arm_sdk_publisher.Write(low_cmd)

# ========== Step 7: 循环 Step 4-6（50 Hz） ==========
thread = RecurrentThread(interval=0.02, target=control_loop)
thread.Start()

# ========== Step 8: 结束时释放控制 ==========
low_cmd.motor_cmd[29].q = 0.0   # kNotUsedJoint = 0 → 释放 arm_sdk
low_cmd.crc = crc.Crc(low_cmd)
arm_sdk_publisher.Write(low_cmd)
```

---

## 八、GMR 数据与 SDK 接口的映射

### 8.1 关节索引对照表

GMR 的 `dof_pos`（29 维）与 SDK 的 `motor_cmd` 索引**完全一致**：

```
GMR dof_pos[i]  ←→  SDK motor_cmd[i]
```

| GMR 关节名 | dof索引 | SDK索引 | 部位 |
|-----------|---------|---------|------|
| `left_hip_pitch_joint` | 0 | 0 | 左腿 |
| ... | 1–5 | 1–5 | 左腿 |
| `right_hip_pitch_joint` | 6 | 6 | 右腿 |
| ... | 7–11 | 7–11 | 右腿 |
| `waist_yaw_joint` | 12 | 12 | 腰部 |
| `waist_roll_joint` | 13 | 13 | 腰部 |
| `waist_pitch_joint` | 14 | 14 | 腰部 |
| `left_shoulder_pitch_joint` | **15** | **15** | 左臂 |
| `left_shoulder_roll_joint` | **16** | **16** | 左臂 |
| `left_shoulder_yaw_joint` | **17** | **17** | 左臂 |
| `left_elbow_joint` | **18** | **18** | 左臂 |
| `left_wrist_roll_joint` | **19** | **19** | 左臂 |
| `left_wrist_pitch_joint` | **20** | **20** | 左臂 |
| `left_wrist_yaw_joint` | **21** | **21** | 左臂 |
| `right_shoulder_pitch_joint` | **22** | **22** | 右臂 |
| `right_shoulder_roll_joint` | **23** | **23** | 右臂 |
| `right_shoulder_yaw_joint` | **24** | **24** | 右臂 |
| `right_elbow_joint` | **25** | **25** | 右臂 |
| `right_wrist_roll_joint` | **26** | **26** | 右臂 |
| `right_wrist_pitch_joint` | **27** | **27** | 右臂 |
| `right_wrist_yaw_joint` | **28** | **28** | 右臂 |

### 8.2 直接赋值

```python
dof_pos = loaded_data["dof_pos"]  # (N, 29) from GMR

# 无需任何索引转换，直接复制
for i in range(15, 29):  # 手臂关节 15~28
    low_cmd.motor_cmd[i].q = float(dof_pos[frame_idx][i])
```

---

## 九、安全注意事项

1. **arm_sdk 通道的 kNotUsedJoint（索引 29）**：
   - `motor_cmd[29].q = 1` → 使能 arm_sdk 控制
   - `motor_cmd[29].q = 0` → 释放控制权，手臂回归默认姿态
   - 必须每秒至少发一次包，否则机器人自动释放控制权（超时保护）

2. **CRC 校验**：必须在 `Write()` 之前调用 `crc.Crc(low_cmd)`，否则指令被静默丢弃

3. **关节角度单位**：SDK 使用**弧度（rad）**，GMR 也输出弧度，无需转换

4. **关节限位**：G1 各关节有硬件限位，超出范围的电机会被保护性禁用。arm_sdk 示例中的范围参考：
   - 肩部 Pitch: ±180°（±π rad）
   - 肘部: 0° ~ −180°（折弯方向）
   - 腕部: ±180°

5. **紧急停止**：`Ctrl+C` 触发 `KeyboardInterrupt`，应立即将 `kNotUsedJoint.q = 0` 并多发几次确保机器人收到
