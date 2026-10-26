# My-Robocup-robot-codes · 双足人形机器人

一台**双足人形机器人**的完整控制代码：STM32F407 主控 + CanMV(K230D) 视觉协处理 + 14 路 ST3215 串口总线舵机。能走直道、转弯、越障，按视觉识别做图形动作。

整机尺寸 **205.6 × 420.2 × 198.7 mm**，55 个零件。

---

## 硬件

| 项 | 参数 |
|---|---|
| 主控 | STM32F407VETx，外部晶振 **16 MHz**，PLLM=8 → 系统时钟 168 MHz |
| 舵机 | ST3215 串口总线舵机 × 14，ID 1–14，控制周期 10 ms |
| 视觉 | CanMV（K230D）跑 MicroPython，负责巡线 / 图形 / 横杆角度 |
| 测距 | HC-SR04 超声 |
| 调试台 | 串口 115200 8N1 |

### 接线

| 功能 | 引脚 | 参数 |
|---|---|---|
| 视觉串口 | UART4：PC10=TX、PC11=RX | 115200 8N1，中断收发 |
| 调试控制台 | USART1：PA9=TX、PA10=RX | 115200 8N1 |
| 舵机总线 | USART2：PD5=TX、PD6=RX | 1 Mbps，经外接适配器 |
| HC-SR04 | PA2=TRIG、PA3=ECHO | 注意 PA3 在本控制板上为 FT 数字输入 |
| 板载按键 | KEY1=PE1、KEY2=PE0 | 10 kΩ 上拉，按下低电平，软件 30 ms 消抖 |
| SG90（图形 6 用） | PA11 | 50 Hz PWM |
| 状态灯 | PE10 | |

> ⚠️ 晶振频率是**控制板硬件属性**。本目录是 16 MHz 版；给 8 MHz 板子烧这个固件会跑飞。8 MHz 版的视觉串口是 USART6（PC6/PC7）、超声是 PA0/PA1。

---

## 目录结构

```
8.0_16MHz/
├── Src/ Inc/              固件源码（14 路舵机控制、步态状态机、串口协议）
├── Actions/               动作帧 JSON —— 唯一的人工编辑入口
├── MDK-ARM/               Keil MDK 工程 HumanoidRobot_8_0.uvprojx
├── Drivers/               ST HAL + CMSIS（第三方，见"许可"）
├── Validation/            动作帧生成器与校验脚本（Python）
├── Vision_8.0/            CanMV 视觉代码（MicroPython）
└── README_5.3…8.0_16MHz.md 各版本的设计说明与改动记录
```

---

## 构建

1. Keil MDK 5 打开 `8.0_16MHz/MDK-ARM/HumanoidRobot_8_0.uvprojx`
2. Build（工程已配好相对路径，克隆下来直接能编）
3. 下载到 STM32F407VETx 目标板

**引脚配置以代码为准，不是 CubeMX 生成的**（仓库里没有 `.ioc`）。时钟与引脚改动集中在 `Src/main.c` 与 `Inc/robot_config.h`。

---

## 动作帧管线（这个工程的核心）

固件里**没有硬编码姿态**。所有动作都走同一条管线：

```
Actions/*.json  ──Validation/generate_*.py──▶  Inc/*_frames.h  ──编译──▶  固件
   （人工编辑）          （Python 生成器）          （C 常量数组）
```

`Inc/*_frames.h` 头部会写下源 JSON 的 **SHA256**，用来锁定"这份头文件是从哪个 JSON 生成的"，防止动作来源被悄悄改掉又没人发现。

生成器与产物的对应关系：

| 生成器 | 源 JSON | 产物 | 用途 |
|---|---|---|---|
| `generate_actions.py` | `my_actionsfinalorigin4.json` | `action_frames.h` | 直走步态（18 帧） |
| `generate_turns.py` | `TURNLEFT/TURNRIGHT/TURNLEFT1.json`（模板：`my_actions4.json`） | `turn_frames.h` | 左/右转与默认左转 |
| `generate_right_turn.py` | `my_actionsfinalTURNRIGHT.json` | `right_turn_frames.h` | 右转纠偏 |
| `generate_obstacle_actions.py` | `my_actionsfinalshort3.json` / `my_actions strideover4.json` | `obstacle_action_frames.h` | 越障小碎步 / 跨杆 |
| `generate_shapes.py` | `1.json` ~ `5.json` | `shape_frames.h` | 图形 1–5 动作 |

改动作的正确姿势：改 JSON（或改生成器）→ 重新生成头文件 → 编译。**不要手改 `*_frames.h`。**

每一帧的字段是 14 路舵机的目标位置（0–4095 计数，中位 2048）、运动时长、停留时长、速度与加速度，以及"是否等待到位"和"是否落地帧"标记。每路舵机的行程限位写在 JSON 的 `limits` 里，并会被生成进头文件的 `k_action_limits`，运行中超出限位即报故障。

---

## 运行状态机

`Src/robot_motion.c` 里是一个显式状态机：

```
IDLE → ENTERING_STAND → STANDING → (MARCHING ⇄ HOLDING) → FAULT
```

- **站立**：先 `s` 进入站姿；站姿保持后允许每路舵机有较小静差（`ACTION_START_TOLERANCE`），从实测位置平滑衔接下一帧。
- **行走**：`g` 从已完成的站姿开始，并按动作组原始时序循环；换重心、落地与所有运动时序都由 JSON 决定，代码不额外压缩。
- **越障三阶段**：普通 → 小碎步（视觉横杆角纠偏对正）→ 越障跨杆。每阶段只在**原有落地帧边界**切入，不新建一套行走。
- **图形动作**：识别到图形 1–6 时锁存请求，在安全帧（图形 1/2/5/6 等右脚前摆落地、双脚着地的稳定帧）才真正开始。
- **安全**：所有目标都过限位与故障检查；行走中按 KEY1 走 `s` 的安全停止逻辑。

---

## 板载按键与调试指令

| 输入 | 作用 |
|---|---|
| KEY1（PE1） | 等同串口 `s`：进入站立姿态；行走中按下则走安全停止逻辑 |
| KEY2（PE0） | 等同串口 `g`：从已完成的站立姿态开始行走并开启视觉控制 |
| `s` | 进入站立姿态 |
| `g` | 开始行走 + 开启视觉控制 |
| `j` | SG90 测试：PA11 输出 50 Hz PWM，约 ±50° 每 500 ms 换向，3 秒后回中位，保持 400 ms 后关 PWM（行走中或正在进入站姿时拒绝执行） |

调试串口用的是 USART1（PA9/PA10，115200）。

---

## 视觉侧（CanMV / MicroPython）

`8.0_16MHz/Vision_8.0/` 是现役视觉代码，跑在 CanMV 板上，通过 UART4 与主板通信（115200）。

- 上位机入口是 `main.py`（由 `build_standalone.py` 把各模块打成一个单文件；`canmv_line_follow_single.py` 与它字节一致，不需要另外上传模块）。
- 协议包：**AA55** 图形识别、**AA56** 巡线/控制、**AA57** 运动反馈、**AA58** 长横杆角度。
- 主要模块：`line_tracker.py` 巡线、`boundary_tracker.py` 边界与右纠偏、`shape_tracker.py` 图形识别、`angle_tracker.py` 横杆角度、`motion_feedback.py` 反馈节流、`lane_protocol.py` 打包。
- 改模块后必须重新跑 `build_standalone.py` 生成 `main.py`，不要只改生成入口。

另外两套视觉代码是**历史与分支**，不进本仓库主线：`Vision5.1`（上一代，配 5.3 固件）、`Vision_red`（平行分支：第二次出弯后开启红木条识别，用于限宽门阶段）。

---

## 版本演进

| 版本 | 说明 |
|---|---|
| [5.3](8.0_16MHz/README_5.3.md) | 起点，六种图形 + 巡线的基线 |
| [6.0](8.0_16MHz/README_6.0.md) | |
| [7.0](8.0_16MHz/README_7.0.md) | 长横杆检测、AA58 角度包 |
| [8.0](8.0_16MHz/README_8.0.md) | |
| [8.0_16MHz](8.0_16MHz/README_8.0_16MHz.md) | 本目录：改 16 MHz 晶振 + 串口引脚调整 + 板载按键 + 图形落脚时机 |

各版本的详细改动、现场调参入口、实机边界条件都写在对应的 README 里，比这份总览细得多。

---

## 已知限制

- 拆解顺序 / 动作时序按现场调试确定，配的是**当前这台机器**的机械零位；换机器人要重标。
- 视觉阈值（右纠偏进入线、窄道判定、红条开启条件）与赛道强相关，换赛道需要重调。
- 动作帧里的位置是 0–4095 计数，不是角度；换算关系取决于舵机零位与减速比，未在本仓库标定。
- `Drivers/` 只保留编译必需的部分（HAL + CMSIS 头）。CMSIS 的 DSP/NN 库与预编译数学库已剔除，本工程不引用它们。

## 许可

固件与视觉代码版权归作者所有。`Drivers/` 下为 STMicroelectronics 的 HAL 驱动（BSD-3-Clause）与 ARM CMSIS（Apache-2.0），各自的 `LICENSE.txt` 请照原样保留。