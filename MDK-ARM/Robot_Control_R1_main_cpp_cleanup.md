# 2026-07-05 Robot_Control_R1 main.cpp 整理记录

本文档用于说明当前 `Robot_Control_R1` 工程的下位机代码结构、CAN 电机分配、SBUS 遥控器通道、GPIO 继电器输出和常用参数修改位置。后期开新对话或交给队友维护时，可以先上传本文档，让协作者快速了解当前工程状态。

## 当前工程结构

当前工程还没有像其他工程那样拆出 `app/` 模块，主要控制逻辑仍集中在 `Core/Src/main.cpp`，SBUS 解析和通道映射放在 `User/sbus`。

| 路径 | 作用 |
| --- | --- |
| `Core/Src/main.cpp` | 主控制逻辑：SBUS 接收调度、底盘、升降台、夹爪、吸夹、CAN 回调、PID 周期、CAN 周期发送 |
| `User/sbus/sbus.cpp` | SBUS 25 字节帧解析，更新 `SBUS_CH.CH1~CH16` |
| `User/sbus/sbus_control.cpp` | 将 SBUS 原始值映射成控制量 `G_SbusValue` |
| `User/sbus/sbus_control.h` | 处理后的 SBUS 控制量结构体 |
| `User/wheel_chassis/dvc_motor.*` | C620/C610 电机类和 PID 控制 |
| `User/wheel_chassis/dvc_motor_dm.*` | 达妙电机控制类 |
| `User/wheel_chassis/drv_can.*` | CAN 管理对象和发送封装 |
| `User/wheel_chassis/alg_pid.*` | PID 算法 |
| `Core/Src/gpio.c` | GPIO 初始化，包含 PD13/PD14/PD15/PH10 继电器输出 |
| `Core/Src/usart.c` | USART1 SBUS、USART2 调试、UART7 陀螺仪、UART8 红外/预留串口 |

## 工程定位

```text
工程目录：
  D:\Robocon\Robot_Control_R1_hongwai\Robot_Control_R1\MDK-ARM

Keil 工程：
  Robot_Control_R1.uvprojx

核心文件：
  Core/Src/main.cpp

工具链：
  Keil + STM32CubeMX + STM32 HAL

主控板：
  大疆 A 板 / STM32F427

遥控器：
  富斯 ST16

接收机：
  富斯 SR8

主控入口：
  USART1 接 SBUS

辅助调试：
  USART2 115200，当前 SBUS/gyro 调试发送语句默认注释
```

## main.cpp 当前职责

`main.cpp` 目前主要保留：

1. HAL 初始化和 `main()` 主循环。
2. USART1 SBUS 单字节接收、帧同步和解析入口。
3. CAN1/CAN2 电机反馈回调。
4. 底盘四个 3508 万向轮控制。
5. 升降台两个 3508 控制。
6. CAN2 0x205 龙门架平动伸缩控制。
7. CAN2 0x203/0x204 吸夹装置翻转控制。
8. CAN1 0x205/0x206 夹爪总成伸缩和翻转状态机。
9. PD13/PD14/PD15/PH10 继电器输出。
10. 达妙电机发送和备用机械臂逻辑。
11. UART7 陀螺仪、UART8 红外/预留串口相关接收框架。
12. 电机 PID 初始化、CAN 初始化、2ms 周期 PID 计算和 CAN 发送。

## main() 调度节奏

主循环中每隔约 2ms 执行一次核心控制：

```text
检查 SBUS 是否有新帧
  -> Sbus_Update()
  -> SbusControl_ProcessData()

2ms 周期：
  -> execute_sbus_motion_commands()
  -> 各电机 TIM_PID_PeriodElapsedCallback()
  -> 达妙电机发送
  -> TIM_CAN_PeriodElapsedCallback()
```

`execute_sbus_motion_commands()` 是遥控器控制的主要入口。

## CAN1 电机分配

CAN1 负责底盘四个 3508、夹爪总成两个 2006，以及一个达妙电机。

| 对象 | CAN ID | 电机/电调 | 用途 | 控制方式 |
| --- | --- | --- | --- | --- |
| `motor1` | `0x201` | M3508 + C620 | 底盘左前 | 角度闭环 |
| `motor2` | `0x202` | M3508 + C620 | 底盘左后 | 角度闭环 |
| `motor3` | `0x203` | M3508 + C620 | 底盘右后 | 角度闭环 |
| `motor4` | `0x204` | M3508 + C620 | 底盘右前 | 角度闭环 |
| `claw_extend_motor` | `0x205` | M2006 + C610 | CH7 夹爪伸缩 | 角度闭环 |
| `claw_flip_motor` | `0x206` | M2006 + C610 | CH7 夹爪翻转 | 角度闭环 |
| `arm_dm_1` | `0x1B/0x0B` | 达妙 | 备用机械臂 | MIT/达妙控制 |

CAN1 回调入口：

```text
CAN1_Call_Back()
```

CAN1 初始化入口：

```text
motor_can_init()
```

## CAN2 电机分配

CAN2 负责升降台、龙门架平动伸缩、吸夹翻转，以及一个达妙电机。

| 对象 | CAN ID | 电机/电调 | 用途 | 控制方式 |
| --- | --- | --- | --- | --- |
| `motor5` | `0x201` | M3508 + C620 | 升降台左侧电机 | 角度闭环 |
| `motor6` | `0x202` | M3508 + C620 | 升降台右侧电机 | 角度闭环 |
| `suction_claw_left_motor` | `0x203` | M3508 + C620 | 吸夹翻转左电机 | 角度闭环 |
| `suction_claw_right_motor` | `0x204` | M3508 + C620 | 吸夹翻转右电机 | 角度闭环，和 0x203 反向 |
| `lift_extend_motor` | `0x205` | M3508 + C620 | 龙门架平动伸缩 | 角度闭环 |
| `arm_dm_2` | `0x1A/0x0A` | 达妙 | 备用机械臂 | MIT/达妙控制 |

CAN2 回调入口：

```text
CAN2_Call_Back()
```

CAN2 初始化入口：

```text
motor_can_init()
```

## GPIO 继电器分配

这些 GPIO 都是高电平触发继电器。`MX_GPIO_Init()` 中当前配置为推挽输出，PD13/PD14/PD15/PH10 为上拉输出，初始输出低电平。

| 引脚 | 当前用途 | 控制通道 | 逻辑 |
| --- | --- | --- | --- |
| `PD13` | 气缸/原 CH8 夹爪继电器 | CH8 | CH8 大于阈值拉高，否则拉低 |
| `PD14` | CH7 夹爪总成夹取继电器 | CH7 | CH7 到夹住/收回阶段时拉高，回到打开阶段拉低 |
| `PD15` | 吸夹装置吸盘继电器 | CH6 | CH6=992 或 192 拉高，CH6=1792 拉低 |
| `PH10` | 吸夹装置夹爪继电器 | CH6 | CH6=192 拉高，其余档位拉低 |

CH6 当前三档含义：

```text
CH6 = 1792:
  吸盘和夹爪都不工作，PD15=0，PH10=0。

CH6 = 992:
  吸盘工作，PD15=1，PH10=0。

CH6 = 192:
  吸盘保持工作，同时夹爪闭合，PD15=1，PH10=1。
```

## USART 分配

| 串口 | 当前配置 | 用途 |
| --- | --- | --- |
| `USART1` | 100000, 9B, Even, 2 stop | SBUS 遥控器输入 |
| `USART2` | 115200, 8N1 | 调试输出，当前主要发送语句已注释 |
| `UART7` | 9600, 8N1 | 陀螺仪接收框架 |
| `UART8` | 115200, 8N1 | 红外/预留通讯框架 |

注意：SBUS 使用 8E2 风格数据，但 STM32 HAL 开偶校验时通常配置为 `UART_WORDLENGTH_9B + UART_PARITY_EVEN`，这里不要被 CubeMX 回滚成 8B。

## SBUS 通道表

SBUS 原始范围按当前代码约定：

```text
最小值：192
中位值：992
最大值：1792
```

当前主要通道：

| 通道 | 用途 | 代码位置 |
| --- | --- | --- |
| CH1 | 底盘左右平移 `Vy` | `SbusControl_ProcessData()` |
| CH2 | 底盘前后平移 `Vx` | `SbusControl_ProcessData()` |
| CH3 | 升降台手动速度 | `LiftPercent` |
| CH4 | 底盘旋转 `W` | `SbusControl_ProcessData()` |
| CH5 | 吸夹翻转开关 | `Arm_Axis4` |
| CH6 | 吸盘/夹爪继电器三档 | `Control_SuctionClawRelay_By_CH6()` |
| CH7 | 夹爪总成状态机 | `Control_ClawAssembly_By_CH7()` |
| CH8 | PD13 继电器 | `Control_ClawRelay_By_CH8()` |
| CH9 | 龙门架平动两点 | `Arm_Axis3` |
| CH10 | 整车使能 | `Aux2` |
| CH11 | CH5 吸夹翻转角度微调 | `Arm_Axis5` |

断连保护：

```text
SBUS_CH.ConnectState = 0 时：
  Vx/Vy/W 清零
  LiftPercent 清零
  Arm_Axis3/4/5 清零
  Aux2 = -100，等效 CH10 失能
```

## CH10 整车使能

CH10 通过 `Aux2` 映射到 `-100~100`。当前主控判断为：

```cpp
if (G_SbusValue.Aux2 >= -50.0f)
{
    // 底盘、升降台、龙门架平动、吸夹翻转进入控制
}
else
{
    // 底盘刹停，目标同步到当前位置，防止重新使能时突跳
}
```

也就是说，CH10 在最低档附近为失能，拨离最低档后进入使能。

注意：CH6、CH7、CH8 的继电器/夹爪总成逻辑是在 CH10 判断前执行的，当前不完全受 CH10 限制。

## 底盘控制

底盘参数：

```cpp
#define MAX_WHEEL_SPEED     25.0f
#define CHASSIS_CONTROL_DT  0.002f
```

摇杆映射：

```text
CH2 -> Vx，前后
CH1 -> Vy，左右
CH4 -> W，旋转
```

`User/sbus/sbus_control.cpp` 中对底盘摇杆做了指数曲线：

```text
expo = 0.65
死区约 8%
```

`omni_move()` 中又做了一阶低通：

```cpp
float alpha = 0.06f;
```

底盘四轮实际布置：

```text
motor1 = 左前
motor2 = 左后
motor3 = 右后
motor4 = 右前
```

四轮目标分配：

```cpp
target_FL = -vx_f + vy_f + w_f * K;
target_FR =  vx_f + vy_f + w_f * K;
target_RL = -vx_f - vy_f + w_f * K;
target_RR =  vx_f - vy_f + w_f * K;
```

目标角累加后写入：

```cpp
motor1 -> target_FL
motor4 -> target_FR
motor2 -> target_RL
motor3 -> target_RR
```

如果底盘方向异常，优先检查：

```text
1. motor1~motor4 的 CAN ID 是否和实际位置一致。
2. CH1/CH2/CH4 映射方向是否符合遥控器习惯。
3. omni_move() 的四轮运动学符号。
4. MAX_WHEEL_SPEED、PID 参数和 alpha 是否过大。
```

## 升降台控制

升降台由 CAN2 的 `motor5/motor6` 控制。

```text
motor5 = 升降台左侧电机，CAN2 0x201
motor6 = 升降台右侧电机，CAN2 0x202
```

当前 CH3 逻辑：

```text
CH3 中位：
  升降台保持当前位置。

CH3 值变大：
  升降台上升。

CH3 值变小：
  升降台下降。
```

核心代码：

```cpp
lift_target_rad -= (G_SbusValue.LiftPercent / 100.0f) * lift_manual_speed * CHASSIS_CONTROL_DT;
motor5.Set_Target_Angle(-lift_target_rad);
motor6.Set_Target_Angle(lift_target_rad);
```

限制参数：

```cpp
#define LIFT_MAX_ANGLE (10.0f * PI)
float lift_manual_speed = 5.0f;
```

`10.0f * PI` 表示输出轴约 5 圈行程。后续如果加重物导致举不起来，优先检查机械传动、供电、电调电流限制，再适当调大升降台 PID 或减小速度指令。

## CH9 龙门架平动伸缩

龙门架平动伸缩电机：

```text
lift_extend_motor
CAN2 0x205
M3508 + C620
```

当前 CH9 两点逻辑：

```text
上电固定起点 -> 自动正转 2 圈，到 CH9=192 对应的伸出固定点。
CH9=192 或 992 -> 保持伸出固定点。
CH9=1792 -> 从伸出固定点反转 3 圈。
```

核心参数：

```cpp
const float lift_extend_direction = 1.0f;
const float lift_extend_home_to_fixed_angle = 2.0f * 2.0f * PI;
const float lift_extend_retract_angle = 3.0f * 2.0f * PI;
```

如果实际方向反了，修改：

```cpp
const float lift_extend_direction = -1.0f;
```

注意：龙门架平动目标更新在 CH10 使能分支内，CH10 失能时目标会同步到当前位置。

## CH5 + CH11 吸夹翻转

吸夹翻转电机：

```text
suction_claw_left_motor  = CAN2 0x203
suction_claw_right_motor = CAN2 0x204
两个电机目标方向相反
```

CH5 负责基础翻转：

```cpp
const float suction_claw_flip_angle = (110.0f / 360.0f) * 2.0f * PI;
```

也就是 CH5 使能后基础翻转 110 度。

CH11 负责在 CH5 使能后微调：

```cpp
const float suction_claw_fine_speed = 0.6f;
const float suction_claw_fine_limit = (30.0f / 180.0f) * PI;
```

逻辑：

```text
CH5 未使能：
  吸夹回到翻转原点，同时清零 CH11 微调量。

CH5 使能：
  目标角 = 110 度基础角 + CH11 累计微调角。

CH11 中位：
  保持当前微调角。

CH11 值增大：
  在当前角度基础上继续正向微调。

CH11 值变小：
  反向微调。

CH11 微调限幅：
  当前限制为 ±30 度。
```

## CH6 吸盘/夹爪继电器

CH6 直接控制 PD15 和 PH10，不通过 `G_SbusValue.Arm_Axis3`。

```text
PD15 = 吸盘继电器
PH10 = 吸夹装置夹爪继电器
```

当前逻辑：

```text
CH6=1792:
  PD15=0，PH10=0，吸盘和夹爪都不工作。

CH6=992:
  PD15=1，PH10=0，吸盘工作。

CH6=192:
  PD15=1，PH10=1，吸盘保持工作，夹爪闭合。
```

断连时：

```text
PD15=0
PH10=0
```

## CH7 夹爪总成状态机

CH7 控制 CAN1 0x205、CAN1 0x206 和 PD14。

```text
claw_extend_motor = CAN1 0x205，夹爪伸缩
claw_flip_motor   = CAN1 0x206，夹爪翻转
PD14              = 夹爪闭合继电器
```

CH7 三档：

```text
CH7=192:
  夹爪松开，伸出状态，翻转回原位。

CH7=992:
  夹爪闭合，伸出状态，翻转回原位。

CH7=1792:
  夹爪保持闭合，先翻转，再延时，再缩回。
```

当前关键参数：

```cpp
const float claw_extend_direction = -1.0f;
const float claw_flip_direction = 1.0f;
const float claw_extend_angle = 2.0f * 2.0f * PI;
const float claw_flip_angle = 1.5f * 2.0f * PI;
const uint32_t high_flip_wait_ms = 1000U;
```

如果 0x205 伸缩方向反了，修改：

```cpp
const float claw_extend_direction = 1.0f;
```

如果 0x206 翻转方向反了，修改：

```cpp
const float claw_flip_direction = -1.0f;
```

## CH8 继电器

CH8 控制 PD13：

```text
CH8 大于 1192 左右：
  PD13=1

否则：
  PD13=0
```

该通道最早用于夹爪继电器，后来改为控制让夹爪上下运动的气缸。因为硬件仍是继电器高电平触发，所以代码逻辑不需要区分夹爪还是气缸。

## PID 参数位置

PID 初始化集中在 `motor_pid_init()`。

### 底盘 PID

```cpp
motor1~motor4.PID_Angle:
  Kp = 14.0f
  输出限幅 = MAX_WHEEL_SPEED
  dt = CHASSIS_CONTROL_DT

motor1~motor4.PID_Omega:
  Kp = 500.0f
  Ki = 300.0f
  积分限幅 = 1000.0f
  输出限幅 = 5500.0f
```

底盘最大轮速：

```cpp
#define MAX_WHEEL_SPEED 25.0f
```

启动太猛时，优先调：

```text
1. MAX_WHEEL_SPEED
2. 底盘角度环 Kp
3. 底盘速度环 Kp
4. omni_move() 中 alpha
```

### 升降台 PID

```cpp
motor5/motor6.PID_Angle:
  Kp = 35.0f
  输出限幅 = 10.0f

motor5/motor6.PID_Omega:
  Kp = 800.0f
  Ki = 50.0f
```

### 龙门架平动 PID

```cpp
lift_extend_motor.PID_Angle:
  Kp = 15.0f
  输出限幅 = 40.0f

lift_extend_motor.PID_Omega:
  Kp = 800.0f
  Ki = 50.0f
```

### 吸夹翻转 PID

```cpp
suction_claw_left_motor/right_motor.PID_Angle:
  Kp = 15.0f
  输出限幅 = 6.0f

PID_Omega:
  Kp = 800.0f
  Ki = 50.0f
```

### CH7 夹爪总成 PID

```cpp
claw_extend_motor.PID_Angle:
  Kp = 15.0f
  输出限幅 = 60.0f

claw_extend_motor.PID_Omega:
  Kp = 500.0f
  Ki = 50.0f
  积分限幅 = 2000.0f

claw_flip_motor.PID_Angle:
  Kp = 15.0f
  输出限幅 = 5.0f

claw_flip_motor.PID_Omega:
  Kp = 700.0f
  Ki = 60.0f
  积分限幅 = 2000.0f
```

## 常用参数修改位置

### 底盘最大速度

位置：`Core/Src/main.cpp`

```cpp
#define MAX_WHEEL_SPEED 25.0f
```

### 底盘启动柔和程度

位置：`omni_move()`

```cpp
float alpha = 0.06f;
```

`alpha` 越小，加速越柔和；越大，响应越快。

### 升降台最大行程

位置：`Core/Src/main.cpp`

```cpp
#define LIFT_MAX_ANGLE (10.0f * PI)
```

### 升降台手动速度

位置：`execute_sbus_motion_commands()`

```cpp
float lift_manual_speed = 5.0f;
```

### 龙门架平动圈数

位置：`execute_sbus_motion_commands()`

```cpp
const float lift_extend_home_to_fixed_angle = 2.0f * 2.0f * PI;
const float lift_extend_retract_angle = 3.0f * 2.0f * PI;
```

### 吸夹翻转角度

位置：`execute_sbus_motion_commands()`

```cpp
const float suction_claw_flip_angle = (110.0f / 360.0f) * 2.0f * PI;
```

### 吸夹翻转微调速度和范围

位置：`execute_sbus_motion_commands()`

```cpp
const float suction_claw_fine_speed = 0.6f;
const float suction_claw_fine_limit = (30.0f / 180.0f) * PI;
```

### CH7 夹爪伸缩/翻转圈数

位置：`Control_ClawAssembly_By_CH7()`

```cpp
const float claw_extend_angle = 2.0f * 2.0f * PI;
const float claw_flip_angle = 1.5f * 2.0f * PI;
```

## 调试流程建议

### 1. 先确认 SBUS

```text
1. USART1 必须是 100000、9B、Even、2 stop。
2. `SBUS_CH.ConnectState` 应为 1。
3. CH1~CH16 原始值应在 192/992/1792 附近变化。
4. USART2 打印默认注释，必要时再临时打开。
```

### 2. 再确认 CH10 使能

```text
1. CH10 最低档时底盘和升降台不应动作。
2. CH10 拨离最低档后，底盘、升降台、CH5、CH9 开始响应。
3. 如果使能瞬间底盘跳动，检查 Chassis_SyncTargetToNow() 和当前位置反馈。
```

### 3. 底盘单独调试

```text
1. 架空底盘。
2. 只小幅拨 CH1/CH2/CH4。
3. 确认前后、左右、旋转方向。
4. 方向反了先查摇杆映射，再查 omni_move() 符号。
```

### 4. 升降台调试

```text
1. 先不放重物。
2. CH3 中位时必须保持当前位置。
3. CH3 变大应上升，变小应下降。
4. 两个电机互相较劲时，检查 motor5/motor6 目标符号。
```

### 5. 龙门架平动调试

```text
1. 确认 CAN2 0x205 单独在线。
2. 上电初始位置要固定。
3. CH10 使能后应先到 CH9=192 固定点，也就是从初始位伸出 2 圈。
4. CH9 切到 1792 后应从固定点反转 3 圈。
5. 方向反了改 lift_extend_direction。
```

### 6. 吸夹翻转调试

```text
1. 确认 CAN2 0x203/0x204 ID 正确。
2. CH5 使能后应翻转到 110 度。
3. 两个电机应反向配合，不应互相打架。
4. CH11 增大继续正调，变小反调。
5. 微调方向反了，改 CH11 累加方向或左右电机目标符号。
```

### 7. CH6 吸盘/夹爪继电器调试

```text
1. CH6=1792 时 PD15/PH10 都应为低。
2. CH6=992 时 PD15 应为高，PH10 为低。
3. CH6=192 时 PD15/PH10 都应为高。
4. 如果继电器是低电平触发，需要整体反相 GPIO 输出。
```

### 8. CH7 夹爪总成调试

```text
1. 先单独确认 CAN1 0x205 伸缩方向。
2. 再确认 CAN1 0x206 翻转方向。
3. CH7=192 应松开并保持伸出。
4. CH7=992 应 PD14 拉高夹住。
5. CH7=1792 应先翻转，等待 1 秒，再缩回。
6. 从 1792 回 992 时，应先伸出，再翻回原位。
7. 从 992 回 192 时，PD14 拉低松开。
```

## 风险点

### CubeMX 回滚配置

曾经出现过 CubeMX 重新生成后把 USART1 数据位、PD13 上下拉等配置带回错误状态。重新生成后重点检查：

```text
USART1:
  BaudRate = 100000
  WordLength = UART_WORDLENGTH_9B
  Parity = UART_PARITY_EVEN
  StopBits = UART_STOPBITS_2

PD13/PD14/PD15/PH10:
  GPIO_MODE_OUTPUT_PP
  GPIO_PULLUP
  初始输出低电平
```

### CAN ID 冲突

```text
同一条 CAN 总线上不能有两个电调使用同一个 ID。
当前 CAN2 的 0x203/0x204 已用于吸夹翻转 M3508。
当前 CAN2 的 0x205 已用于龙门架平动 M3508。
当前 CAN1 的 0x205/0x206 已用于 CH7 夹爪总成 M2006。
```

### 上电零点

当前多个机构以上电时电机当前位置作为零点或固定起点：

```text
升降台 motor5/motor6
龙门架平动 lift_extend_motor
吸夹翻转 suction_claw_left/right
CH7 夹爪伸缩和翻转
```

上电前机械位置必须固定，否则代码中的“2 圈、3 圈、110 度”等目标会跟着整体偏移。

### 机械限位

第一次测试新的圈数不要直接放大：

```text
1. 先架空或拆负载。
2. 把圈数临时改小，例如 0.2 圈。
3. 确认方向正确后再恢复实际圈数。
4. 任何机构接近硬限位时，先停机，不要靠 PID 硬顶。
```

### 继电器触发电平

当前代码按高电平触发继电器写：

```text
GPIO_PIN_SET = 继电器吸合
GPIO_PIN_RESET = 继电器释放
```

如果换成低电平触发继电器，需要把对应 `HAL_GPIO_WritePin()` 输出逻辑反过来。

## 后续建议拆分

当前 `main.cpp` 已经比较大，后续如果继续整理，建议按下面顺序拆：

1. `chassis_control.h/.cpp`：底盘万向轮运动学、目标同步、速度限幅。
2. `sbus_motion.h/.cpp`：`execute_sbus_motion_commands()` 和各通道动作分发。
3. `lift_control.h/.cpp`：升降台 motor5/motor6 手动速度控制。
4. `lift_extend_control.h/.cpp`：CAN2 0x205 龙门架平动两点状态。
5. `suction_claw_control.h/.cpp`：CAN2 0x203/0x204 翻转和 CH6 继电器。
6. `claw_assembly_control.h/.cpp`：CH7 夹爪总成状态机。
7. `board_debug.h/.cpp`：USART2 调试输出、UART7/UART8 状态打印。

因为当前大量使用 C++ 电机类，建议继续拆成 `.cpp/.h`，不要强行改成 `.c/.h`。

