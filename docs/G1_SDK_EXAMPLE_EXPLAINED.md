# G1 官方上肢运动脚本详解

## 一、预设动作是否开源？

### 结论：不开源

`g1_arm_action_example.py` 中展示的预设动作（shake hand、hug、dance 等 15 个动作）通过 RPC 调用执行：

```python
armAction_client.ExecuteAction(action_map.get("shake hand"))  # 发送 action_id=27
```

底层调用的是机器人固件内部的 RPC 服务（API ID = 7106），**动作轨迹存储在机器人控制器固件中，外部无法获取关节级轨迹数据**。`g1_arm_action_client.py` 只是发送一个整数 ID 给机器人：

```python
# 源码：g1_arm_action_client.py
def ExecuteAction(self, action_id: int):
    p = {"data": action_id}
    parameter = json.dumps(p)
    code, data = self._Call(ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION, parameter)
    return code
```

### 与此对比：GMR 提取的动作是完全透明的

| 对比维度 | 官方预设动作 | GMR 提取的动作 |
|---------|------------|--------------|
| 轨迹数据 | ❌ 闭源，固件内部 | ✅ 完全开放，dof_pos 数组 |
| 可修改 | ❌ 不可编辑 | ✅ 每帧每个关节值都可修改 |
| 可泛化 | ❌ 仅限预设的 15 个动作 | ✅ 任何视频动作均可提取 |
| 调用方式 | RPC（发送 ID） | DDS 通道（发送关节角度） |

---

## 二、g1_arm7_sdk_dds_example.py 逐段详解

这是官方提供的 **上肢关节级控制示例**，通过 DDS `rt/arm_sdk` 通道直接向 G1 发送关节角度指令。与 `replay_arms_on_g1.py` 使用同一套底层接口。

### 2.1 完整代码结构

```
G1JointIndex 类       → 关节索引常量定义（0~29）
Custom.__init__()      → 初始化参数、创建 LowCmd_ 消息
Custom.Init()          → 创建 DDS 发布/订阅通道
Custom.Start()         → 等待机器人就绪，启动 50Hz 控制线程
Custom.LowStateHandler() → 接收机器人状态反馈（回调）
Custom.LowCmdWrite()   → 核心：每 20ms 执行一次的控制循环（4 阶段）
main()                 → 入口：初始化 DDS、启动、等待完成
```

### 2.2 G1JointIndex — 关节索引常量

```python
class G1JointIndex:
    # 左腿 (6个)
    LeftHipPitch = 0      # 髋俯仰
    LeftHipRoll = 1       # 髋侧摆
    LeftHipYaw = 2        # 髋偏航
    LeftKnee = 3          # 膝
    LeftAnklePitch = 4    # 踝俯仰
    LeftAnkleRoll = 5     # 踝侧摆

    # 右腿 (6个)
    RightHipPitch = 6     # ...同上
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11

    # 腰部 (3个)
    WaistYaw = 12         # 腰偏航
    WaistRoll = 13        # 腰侧摆 (29dof版本无效)
    WaistPitch = 14       # 腰俯仰 (29dof版本无效)

    # 左臂 (7个)
    LeftShoulderPitch = 15   # 肩俯仰
    LeftShoulderRoll = 16    # 肩侧摆
    LeftShoulderYaw = 17     # 肩偏航
    LeftElbow = 18           # 肘
    LeftWristRoll = 19       # 腕侧摆
    LeftWristPitch = 20      # 腕俯仰 (23dof版本无效)
    LeftWristYaw = 21        # 腕偏航 (23dof版本无效)

    # 右臂 (7个)
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27     # (23dof版本无效)
    RightWristYaw = 28       # (23dof版本无效)

    kNotUsedJoint = 29       # 控制字（非物理关节）
```

### 2.3 初始化参数

```python
class Custom:
    def __init__(self):
        self.control_dt_ = 0.02       # 控制周期 20ms = 50Hz
        self.duration_ = 3.0          # 每个阶段的持续时间 3秒
        self.kp = 60.0                # 位置增益（比例系数）
        self.kd = 1.5                 # 速度增益（阻尼系数）
        self.dq = 0.0                 # 目标速度（始终为0，点到点运动）
        self.tau_ff = 0.0             # 前馈力矩（始终为0）

        # 目标姿态：手臂上举
        self.target_pos = [
            0., π/2,  0., π/2,  0., 0., 0.,   # 左臂: 肩Pitch=0, 肩Roll=π/2, 肩Yaw=0,
                                                #       肘=π/2, 腕Roll=0, 腕Pitch=0, 腕Yaw=0
            0., -π/2, 0., π/2,  0., 0., 0.,   # 右臂: 肩Pitch=0, 肩Roll=-π/2, ...
            0, 0, 0                            # 腰部: 全部0
        ]
```

`target_pos` 定义了目标姿态的 17 个关节值：左臂 7 个 + 右臂 7 个 + 腰部 3 个。例如 `π/2`（≈1.57 rad ≈ 90°）表示该关节旋转 90 度。

### 2.4 LowCmdWrite — 四阶段控制循环（核心）

这是 50Hz 控制循环的回调函数，通过 `self.time_` 追踪已执行时间，划分为 4 个阶段：

```
时间轴:  0s ────── 3s ────── 9s ─────── 18s ────── 21s
阶段:  [ Stage 1 ][  Stage 2  ][  Stage 3  ][ Stage 4 ]
        归零过渡     上举保持      归零过渡      释放控制
```

---

#### Stage 1（0 ~ 3s）：平滑过渡到零位

```python
if self.time_ < self.duration_:
    # 使能 arm_sdk 控制
    self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

    for i, joint in enumerate(self.arm_joints):
        ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)

        self.low_cmd.motor_cmd[joint].tau = 0.0
        self.low_cmd.motor_cmd[joint].q =
            (1.0 - ratio) * self.low_state.motor_state[joint].q
        self.low_cmd.motor_cmd[joint].dq = 0.0
        self.low_cmd.motor_cmd[joint].kp = self.kp
        self.low_cmd.motor_cmd[joint].kd = self.kd
```

**逐行解释**：

| 变量 | 含义 |
|------|------|
| `self.time_` | 累计时间，每次 +0.02s |
| `self.duration_` | 阶段时长 = 3.0s |
| `ratio` | 插值系数，从 0 线性增长到 1 |
| `self.low_state.motor_state[joint].q` | 机器人**当前实际**关节角度（从状态反馈读取） |

**核心公式**：
```
q_target = (1 - ratio) × q_current
```
- 当 `ratio = 0`（起始）：`q_target = q_current`（保持当前位置不动）
- 当 `ratio = 0.5`（1.5s）：`q_target = 0.5 × q_current`（走到一半）
- 当 `ratio = 1`（3.0s）：`q_target = 0`（到达零位）

**设计意图**：从机器人当前姿态**平滑过渡**到零位（所有关节角度 = 0），避免突然跳变。

---

#### Stage 2（3 ~ 9s）：从零位过渡到目标姿态并保持

```python
elif self.time_ < self.duration_ * 3:
    for i, joint in enumerate(self.arm_joints):
        ratio = np.clip(
            (self.time_ - self.duration_) / (self.duration_ * 2),
            0.0, 1.0
        )
        self.low_cmd.motor_cmd[joint].q =
            ratio * self.target_pos[i] + (1.0 - ratio) * self.low_state.motor_state[joint].q
```

**ratio 计算**：
```
ratio = (time - 3.0) / 6.0
```
- 在 Stage 2 的 6 秒（3s~9s）内从 0 → 1

**核心公式**：
```
q_target = ratio × target_pos[i] + (1 - ratio) × q_current
```
- 当 `ratio = 0`（3s）：`q_target = q_current`（从当前位置出发）
- 当 `ratio = 1`（9s）：`q_target = target_pos[i]`（到达目标姿态）
- 中间线性插值

---

#### Stage 3（9 ~ 18s）：从目标姿态回到零位

```python
elif self.time_ < self.duration_ * 6:
    for i, joint in enumerate(self.arm_joints):
        ratio = np.clip(
            (self.time_ - self.duration_ * 3) / (self.duration_ * 3),
            0.0, 1.0
        )
        self.low_cmd.motor_cmd[joint].q =
            (1.0 - ratio) * self.low_state.motor_state[joint].q
```

**核心公式**：
```
q_target = (1 - ratio) × q_current
```
与 Stage 1 完全相同的逻辑，从目标姿态平滑归零。

---

#### Stage 4（18 ~ 21s）：释放 arm_sdk 控制

```python
elif self.time_ < self.duration_ * 7:
    for i, joint in enumerate(self.arm_joints):
        ratio = np.clip(
            (self.time_ - self.duration_ * 6) / self.duration_,
            0.0, 1.0
        )
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = (1 - ratio)
```

**核心操作**：
```
kNotUsedJoint.q = 1 - ratio
```
- `ratio=0` → `kNotUsedJoint.q = 1`（保持 arm_sdk 使能）
- `ratio=1` → `kNotUsedJoint.q = 0`（释放 arm_sdk，机器人恢复默认姿态）

---

#### 最后：发送指令

```python
self.low_cmd.crc = self.crc.Crc(self.low_cmd)  # 计算 CRC32 校验和
self.arm_sdk_publisher.Write(self.low_cmd)      # 通过 DDS 发送
```

---

## 三、MotorCmd_ 各字段详解

```python
@dataclass
class MotorCmd_:
    mode: uint8     # 电机模式
    q: float32      # 目标关节角度
    dq: float32     # 目标关节速度
    tau: float32    # 前馈力矩
    kp: float32     # 位置增益
    kd: float32     # 速度增益
    reserve: uint32 # 保留字段
```

### 3.1 `q` — 目标关节角度（Target Position）

| 属性 | 值 |
|------|-----|
| 单位 | **弧度（rad）** |
| 范围 | 各关节不同，由硬件限位决定 |
| 含义 | 希望电机到达的角度位置 |

这是**最核心**的字段。在 GMR 回放场景中，每帧从 `dof_pos[frame_idx][joint_idx]` 读取的值直接赋给 `q`。

**与 GMR 的兼容性**：GMR 输出的是弧度，SDK 期望的也是弧度，无需单位转换。

---

### 3.2 `dq` — 目标关节速度（Target Velocity）

| 属性 | 值 |
|------|-----|
| 单位 | **弧度/秒（rad/s）** |
| 通常取值 | `0.0` |
| 含义 | 希望电机达到的速度 |

在本示例和 GMR 回放中始终设为 `0.0`，因为我们走的是**位置控制模式**（`mode=0`）：只指定目标位置，让 PD 控制器自动处理速度。

> 如果 `mode=1`（速度模式），则 `q` 无意义，电机直接跟踪 `dq` 速度指令。

---

### 3.3 `kp` — 位置增益（Proportional Gain）

| 属性 | 值 |
|------|-----|
| 单位 | 无量纲（Nm/rad 等效） |
| arm_sdk 默认 | 40 ~ 60 |
| 含义 | 位置误差的放大系数 |

**PD 控制公式**（电机控制器内部）：
```
τ_motor = kp × (q - q_actual) + kd × (dq - dq_actual) + tau
           └── 比例项 ──┘       └── 阻尼项 ──┘        └── 前馈项
```

**kp 的效果**：
- `kp` 越大 → 跟踪越"硬"，位置误差修正越快
- `kp` 过大 → 震荡、抖动甚至不稳定
- `kp` 过小 → 跟踪迟钝，手臂"软"

```
类比：弹簧的刚度系数
```

---

### 3.4 `kd` — 速度增益（Derivative/Damping Gain）

| 属性 | 值 |
|------|-----|
| 单位 | 无量纲（Nm·s/rad 等效） |
| arm_sdk 默认 | 1.0 ~ 1.5 |
| 含义 | 速度误差（实际是速度本身）的阻尼系数 |

**kd 的效果**：
- `kd` 越大 → 运动越"粘滞"，抑制震荡
- `kd` 过大 → 运动迟钝，响应慢
- `kd` 过小 → 容易震荡，过冲

```
类比：阻尼器的粘性系数
```

**kp/kd 调参经验**：

| 场景 | kp | kd |
|------|-----|-----|
| arm_sdk 运动回放（安全优先） | 40 | 1.0 |
| 需要快速响应 | 60 | 1.5 |
| 小关节（腕部），防抖动 | 20 | 0.5 |

---

### 3.5 `tau` — 前馈力矩（Feedforward Torque）

| 属性 | 值 |
|------|-----|
| 单位 | **牛·米（Nm）** |
| arm_sdk 默认 | `0.0` |
| 含义 | 直接加到电机上的额外力矩 |

在 arm_sdk 示例中始终为 0。前馈力矩用于**补偿已知扰动力**（如重力补偿、负载补偿）：

```
τ_total = kp×(q - q_actual) + kd×(0 - dq_actual) + tau_ff
                                           └── 补偿重力 ──┘
```

GMR 回放不需要设置 `tau`，因为 PD 控制器本身就能补偿重力（只要 kp 足够大）。

---

### 3.6 `mode` — 电机控制模式

| 值 | 含义 |
|----|------|
| `0` | 位置模式：跟踪 `q`（PD 控制），忽略 `dq` |
| `1` | 速度模式：跟踪 `dq`，忽略 `q` |

arm_sdk 始终使用 `mode=0`（位置控制）。

---

### 3.7 `kNotUsedJoint`（索引 29）— arm_sdk 控制字

```python
G1JointIndex.kNotUsedJoint = 29
```

这个"关节"不是物理电机，而是 arm_sdk 通道的**使能/失能开关**：

| 值 | 含义 |
|----|------|
| `motor_cmd[29].q = 1.0` | 使能 arm_sdk，手臂跟随下发的 `q` 指令 |
| `motor_cmd[29].q = 0.0` | 失能 arm_sdk，手臂回到默认姿态（下垂） |

**超时保护**：如果超过约 1 秒没有收到新的 arm_sdk 指令，机器人自动释放控制（安全机制）。

---

## 四、完整控制流程时序图

```
时间轴 (秒):     0         3                   9                   18        21
                │ Stage 1 │     Stage 2       │     Stage 3       │ Stage 4 │
                │  归零    │   举臂+保持       │      归零          │ 释放    │
                │          │                  │                   │         │
ratio:          │ 0 → 1   │     0 → 1        │      0 → 1        │ 0 → 1   │
                │          │                  │                   │         │
q_target公式:   │ (1-r)×cur│ r×target+(1-r)×cur│   (1-r)×cur       │ —       │
                │          │                  │                   │         │
kNotUsed[29].q: │    1     │        1         │        1          │ 1 → 0   │
                │          │                  │                   │         │
手臂姿态:        └─当前→零─┘──零→上举→保持───┘──上举→零──────────┘─零→下垂─┘
```

---

## 五、与 GMR 回放脚本的对比

| 特性 | g1_arm7_sdk_dds_example.py | replay_arms_on_g1.py |
|------|---------------------------|---------------------|
| 轨迹来源 | 硬编码的固定目标角度 | GMR 提取的视频动作（885帧） |
| 控制模式 | 3 阶段线性插值 | 逐帧跟踪 GMR 关节序列 |
| 关节数量 | 17 个（arm+waist） | 14 个（仅 arm，waist 冻结） |
| kp/kd | 全局固定 | 可命令行调节 |
| 安全过渡 | 从当前位置出发 | 1 秒平滑过渡 + 结束时回位 |

---

## 六、控制架构深度分析

### 6.1 控制方式本质：半闭环 + 时间驱动的轨迹生成

`g1_arm7_sdk_dds_example.py` 的控制方式可以概括为：**指定一个 duration 和目标位置，在这段时间内生成从当前实际位置到目标位置的平滑轨迹**。

但这**不是**一个纯前馈的开环轨迹。每 20ms 的控制周期，轨迹从机器人的**实际当前位置**（编码器反馈）重新计算插值起点：

```python
# Stage 2 的核心公式（每 20ms 执行一次）
ratio = time_elapsed / total_duration                     # 纯时间驱动
q_target = ratio × target_pos + (1-ratio) × q_actual     # 从实际位置出发！
#                                         ^^^^^^^^
#                    这是电机编码器反馈的真实角度，不是上一帧下发的指令值
```

这里的关键在于 `q_actual` 是 `low_state.motor_state[joint].q`——**从机器人收到的实际关节角度反馈**，不是 PC 端自己记录的"上一帧指令值"。这构成了一个带位置反馈的轨迹生成机制：

```
时间驱动的 ratio + 编码器反馈的起点 = 半闭环轨迹生成
```

#### 举例：机器人因负载重而跟不上了

```
时间:  0s        1s        2s        3s        4s        5s        6s
ratio: 0.0       0.17      0.33      0.50      0.67      0.83      1.0

纯前馈轨迹（无反馈）：
q_cmd: 0° → 15° → 30° → 45° → 60° → 75° → 90°   ← 不管实际到没到

本代码（从实际位置重算）：
q_cmd: 0° → 13° → 25° → 38° → 52° → 68° → 85°   ← 因为电机滞后（实际只到 30°），
                                                    每帧从落后位置重新出发，
                                                    整个轨迹的加速度需求被稀释
```

**这不是绝对的优点**——ratio 仍然按时走到 1.0，如果机器人严重滞后，最后一帧依然会发出 `q_cmd = target_pos`，只是整个过渡过程变得比预期更平缓。

---

### 6.2 最终关节角度校验：完全不存在

代码中**没有任何位置到达校验**。阶段切换的唯一判断条件是时间：

```python
if self.time_ < self.duration_:           # Stage 1 → 时间到了就切
elif self.time_ < self.duration_ * 3:     # Stage 2 → 时间到了就切
elif self.time_ < self.duration_ * 6:     # Stage 3 → 时间到了就切
```

以下校验逻辑**不存在**：

```python
# ❌ 这些代码不存在于官方示例中
if abs(q_actual - target_pos) < 0.01:     # 实际角度是否到达目标？
    stage_complete = True

if tracking_error > 0.5:                  # 跟踪误差是否过大？
    abort()                                # 触发安全停止
```

**这是有意为之的简化设计**：官方示例的目的是演示 SDK 如何下发指令，不是做一个生产级的运动控制器。在 `kp=60` 且手臂空载的正常情况下，3 秒足够 G1 完成 90° 的运动，跟踪误差在可接受范围内。但对于 GMR 视频动作回放等精确应用，你需要理解这个局限。

---

### 6.3 时间设置与物理能力的匹配问题

#### 核心问题：duration 过小会发生什么？

**会的**——如果给定的 duration 小于电机的物理响应时间，机器人会没完成运动就进入下一阶段。

用具体数字说明：

```
假设把 duration 从 3.0 改成 0.5（要求手臂在 0.5s 内转 90°）

控制侧时间轴:
      0s ─────────── 0.5s ────────────────── 3.5s
      [  Stage 1    ][       Stage 2           ]
      ratio: 0→1     ratio: 0 ──────────→ 1

电机物理响应:
      0s ───────────────────────────── 1.5s
      [  电机加速+减速，实际转 90°约需 1.2s   ]

结果:
      - 0.5s 时 ratio 已到 1.0，q_cmd = target_pos = 90°
      - 但此时实际角度可能只到 ~20°
      - Stage 2 的前几帧：从 20° 重新出发，ratio 从 0 重新开始计数
      - 整个"平滑运动"的意图完全被破坏
```

#### duration 与 kp 的交互影响

| duration | kp | 实际效果 |
|----------|-----|---------|
| 3.0s | 60 | ✅ 跟踪良好，手臂平滑到达目标 |
| 1.0s | 60 | ⚠️ 有较大跟踪误差，但最终能到位 |
| 0.5s | 60 | ❌ 严重滞后，轨迹变形，可能震荡 |
| 0.5s | 200 | ❌ 可能不稳定（震荡甚至失控） |

**duration 设置的底线**：必须大于电机的物理响应时间。G1 臂部关节从静止转动 90° 大约需要 1~1.5 秒（受 kp、负载、关节限速约束）。

---

### 6.4 整体控制架构图

```
┌──────────────────────────────────────────────────────────┐
│  PC 端（50Hz 控制循环）                                    │
│                                                          │
│  ratio = f(time)              ← 纯时间驱动                │
│  q_cmd = lerp(q_actual, target, ratio)  ← 带编码器反馈    │
│  阶段切换 = time > threshold   ← 无位置校验                │
│                                                          │
│  ⚠️ 责任边界：只负责"发指令"，不负责"确认执行完"           │
└──────────────────────┬───────────────────────────────────┘
                       │ DDS (rt/arm_sdk)
                       ▼
┌──────────────────────────────────────────────────────────┐
│  机器人端（~1kHz 电机控制器）                               │
│                                                          │
│  τ = kp×(q_cmd - q) + kd×(0 - dq) + tau   ← PD 跟踪      │
│  硬件限位保护、过流保护、超时释放                            │
│                                                          │
│  ⚠️ 责任边界：只负责"跟踪当前指令"，不负责"判断运动是否完成" │
└──────────────────────────────────────────────────────────┘
```

**总结**：官方示例的控制架构是一个"半闭环 + 时间驱动"的轨迹生成器，适合演示和简单动作。对于 GMR 视频动作回放场景，由于 GMR 输出的 50 FPS 数据本身就是逐帧目标位置序列（不存在阶段切换问题），每帧独立发给 PD 控制器跟踪即可，这个架构完全够用。

---

## 七、两种脚本的核心差异

### 7.1 一句话总结

| 脚本 | 轨迹来源 | 核心逻辑 |
|------|---------|---------|
| `g1_arm7_sdk_dds_example.py` | 给定**一个**最终位置 + 一个 duration | 在 duration 内用 lerp 从当前位置插值到目标 |
| `replay_arms_on_g1.py` | GMR 输出的**每一帧**关节角度（885 帧） | 每帧直接读取目标值下发，不需要插值 |

### 7.2 形象类比

```
g1_arm7_sdk_dds_example.py（官方）:
   "从这里到那里，给你 3 秒钟"  → 自己算中间每一步该在哪
   
   起点(当前位置) ──lerp──→ 终点(target_pos)
   ratio: 0.0 ─────────────────→ 1.0
   q_cmd = lerp(q_actual, target, ratio)


replay_arms_on_g1.py（GMR 回放）:
   "这是每一帧的目标位置，照做就行"  → 数据已经包含了每一步
   
   帧0 ──→ 帧1 ──→ 帧2 ──→ ... ──→ 帧884
   q_cmd = dof_pos[frame_idx][joint]
```

### 7.3 关键差异对照表

| 维度 | g1_arm7_sdk_dds_example | replay_arms_on_g1 |
|------|------------------------|-------------------|
| **轨迹来源** | 1 个硬编码 target_pos | GMR 提取的 885 帧关节序列 |
| **每帧 q 的计算** | `lerp(q_actual, target, ratio)` | `dof_pos[frame_idx][joint]`（直接读取） |
| **是否依赖编码器反馈** | ✅ 每帧读取 `low_state.motor_state[joint].q` | ❌ 不读反馈，纯开环下发 |
| **需要插值？** | ✅ 需要（只有起终点，中间自己算） | ❌ 不需要（每一帧都是精确目标） |
| **阶段切换逻辑** | 4 阶段，基于 `time_` 判断 | 4 阶段，基于 `elapsed` 判断 |
| **duration 来源** | 硬编码 `self.duration_ = 3.0` | `total_frames / fps / speed`（数据驱动） |
| **适用场景** | 预设的简单动作（挥手、举手等） | 任意视频提取的运动轨迹 |

### 7.4 为什么 GMR 不需要插值？

GMR 输出的 `dof_pos` 已经是 **50 FPS × 每帧 29 个关节角度** 的稠密序列。每一帧之间的时间间隔（20ms）与 arm_sdk 的控制周期（20ms）完全一致。这意味着：

```
数据:     帧0   帧1   帧2   帧3   ...   帧884
时间:     0ms   20ms  40ms  60ms      17680ms
           │     │     │     │           │
SDK下发:   q₀    q₁    q₂    q₃         q₈₈₄
```

**帧与帧之间已经足够稠密（50Hz），电机 PD 控制器本身就会平滑跟踪，不需要在 PC 端再做插值。** 这就是 replay 脚本可以直接 `q = dof_pos[frame_idx][joint]` 的原因。

### 7.5 二者的工程定位

```
g1_arm7_sdk_dds_example.py
  └── 教学演示：展示 SDK 使用方法，动作本身不重要
      核心价值：告诉你 Init() → Start() → LowCmdWrite() → CRC → Write() 的流程

replay_arms_on_g1.py
  └── 生产工具：将任意视频动作部署到真实机器人
      核心价值：把 GVHMR → GMR → dof_pos → motor_cmd 这条链路跑通
```
