# Eyes Rehabilitation Device 固件说明

本工程是基于 STM32H723 的眼康复训练设备固件。系统通过语音命令启动训练流程，使用 BMI088 姿态传感器检测头部姿态，控制舵机和激光点完成视觉训练，并通过按键、蜂鸣器、LED、WS2812 和语音模块提供反馈。当前版本还加入了 SBUS 输入，用于辅助归零、在线状态监测和调试。

## 工程结构

| 路径 | 作用 |
| --- | --- |
| `Core/` | STM32CubeMX 生成的 HAL 初始化代码，包括时钟、GPIO、DMA、SPI、TIM、UART 和中断入口。 |
| `App/` | 应用层训练状态机，负责语音命令解析、训练模式切换、安全暂停、训练反馈和舵机视野标定。 |
| `Device/` | 设备驱动层，包括 BMI088、舵机、语音、激光、按键、LED、蜂鸣器、WS2812、头部姿态分析和 SBUS。 |
| `Drivers/` | STM32H7 CMSIS 与 HAL 驱动库。 |
| `MDK-ARM/` | Keil MDK 工程文件和编译输出目录。 |
| `Eyes_Rehabilitation_Device.ioc` | STM32CubeMX 工程配置文件。 |

## 主程序流程

程序入口位于 `Core/Src/main.c`。启动后执行 HAL 初始化、系统时钟配置和外设初始化，然后依次初始化 BMI088、舵机、激光、按键、LED、头部姿态分析、训练状态机、蜂鸣器、WS2812、语音模块和 SBUS。

初始化阶段包含几个硬件自检动作：

- `BMI088_init()` 循环等待 IMU 初始化成功，然后调用 `BMI088_AsyncStart()` 启动异步采样。
- 舵机执行 `Servo_Test()`，用于确认 X/Y 两轴 PWM 输出正常。
- 激光执行 `Laser_Test()`，用于确认激光控制输出正常。
- 蜂鸣器、激光和语音模块依次提示初始化完成。
- 当前代码中保留了 PA2 患者按键测试流程，短按后才进入主循环。

进入主循环后，程序约每 10 ms 执行一次核心任务：

1. `SBUS_Update()` 检查遥控输入是否在线。
2. `BMI088_Task()` 处理 BMI088 异步采样任务。
3. 检查 SBUS 的 SWB 中位事件，如触发则重置 BMI088 姿态参考并重新初始化头部分析器。
4. 读取 BMI088 欧拉角，调用 `HeadTracker_Update()` 更新头部姿态分析。
5. 根据姿态角判断是否进入报警状态，控制蜂鸣器和 WS2812。
6. 扫描按键，调用 `App_Run()` 推进训练状态机。
7. 每 100 ms 通过 UART1 输出一帧 VOFA 短调试数据。

## 应用状态机

训练逻辑位于 `App/train_modes.c`，核心接口为：

- `App_Init()`：初始化训练状态机和训练记录。
- `App_Run()`：主循环中周期调用，处理语音命令和训练状态。
- `App_GetState()`：获取当前系统状态。
- `App_GetMode()`：获取当前训练模式。

主要状态流转如下：

```text
SYS_IDLE_VOICE -> SYS_CALIBRATE -> SYS_TRAIN -> SYS_FEEDBACK -> SYS_IDLE_VOICE
```

特殊状态包括：

- `SYS_CALIBRATE`：每次正式进入训练前都会先进入手动姿态校准。系统关闭刺激、舵机回中并播放校准等待提示；患者调整好坐姿和板子安装姿态后短按 PA2，程序调用 `BMI088_euler_init()` 和 `HeadTracker_Init()`，再进入当前训练模式。
- `SYS_PAUSE`：训练过程中检测到异常姿态后进入暂停。暂停时关闭激光，舵机回中，语音和蜂鸣器提示患者调整姿态。姿态连续正常 3 秒后自动恢复训练，并补偿暂停期间的训练时间基准，避免继续后直接超时进入反馈。
- `SYS_CALIB_SERVO`：语音命令“标定模式”触发，用 PA2 按键标记视野边界，更新训练用舵机角度范围。

语音命令分为两类：

- 命令词：选择训练模式或进入标定模式。五种训练模式名当前在空闲、训练、暂停和反馈状态下都可用于直接切换到指定模式。
- 唤醒词区命令：用于暂停、继续、重新开始、跳过当前训练，以及“你好小盈”语音监听窗口。

控制词当前语义如下：

- “重新开始”：在空闲、训练、暂停和反馈状态下都有效，使用当前 `train_mode` 进入 `SYS_CALIBRATE`，PA2 确认后从当前模式第 0 步重新开始。
- “跳过这个”：在空闲、训练、暂停和反馈状态下都有效，先切换到下一个模式，再进入 `SYS_CALIBRATE`，PA2 确认后开始新模式。
- “你好小盈”：在训练或暂停状态下进入 5 秒静音监听窗口，系统保存现场、停止刺激、冻结计时并进入 `SYS_PAUSE`，给用户留出说后续命令的时间；5 秒内没有后续命令则自动恢复训练。
- “暂停训练/停止训练”：训练中进入可恢复暂停；在“你好小盈”监听窗口内收到该命令时，关闭 5 秒自动恢复窗口并保持暂停，等待“继续训练”或其他命令。
- “继续训练”：只在 `SYS_PAUSE` 有效，恢复暂停前舵机/激光/LED 状态并补偿暂停时间。
- 五种训练模式名：在空闲、训练、暂停和反馈状态下都有效，收到后直接停止当前刺激、切换到指定模式并进入 `SYS_CALIBRATE`，等待 PA2 确认后开始新模式。

## 五种训练模式

| 模式 | 名称 | 主要逻辑 |
| --- | --- | --- |
| `MODE_A_FIXATION` | 注视稳定训练 | 激光固定点亮 15 秒，记录头部稳定性，根据稳定性给出语音反馈。该模式不因姿态异常进入全局暂停，只周期提醒患者保持头部稳定。 |
| `MODE_B_SACCADE` | 扫视训练 | 激光在 4 个位置随机跳变 8 次，患者看到光点后按 PA2，记录正确次数和反应时间。 |
| `MODE_C_PURSUIT` | 平稳追踪训练 | 激光平滑移动到 8 个关键点，每到一个关键点等待 2 秒 PA2 确认，记录正确率、反应时间和头部代偿情况。 |
| `MODE_D_FOCUS` | 视觉聚焦训练 | 近距 LED 与远距激光交替显示，每 5 秒切换一次，共 5 轮。 |
| `MODE_E_NEGLECT` | 空间忽略训练 | 左右视野交替出现激光，患者按 PA2 确认发现光点，用于空间忽略训练。 |

训练结束后进入 `SYS_FEEDBACK`，系统根据训练记录播报结果；完成 5 个模式后会播报全部训练完成提示。

## 安全与姿态检测

头部姿态分析位于 `Device/HeadTracker/`。BMI088 输出欧拉角后，`HeadTracker_Update()` 会更新头部姿态、头部稳定性、代偿性转头和安全状态。

当前主循环和训练状态机都使用垂直安装语义判断姿态：

- `roll`：点头/低头方向。
- `pitch`：左右转头方向。
- `yaw`：侧倾方向。

报警阈值：

- `|roll| > 20 deg` 触发报警。
- `|pitch| > 20 deg` 触发报警。
- `|yaw| > 30 deg` 触发报警。

报警刚出现时蜂鸣器响两声；报警期间 WS2812 红色闪烁。正常状态下 WS2812 执行绿色呼吸灯效果。训练中发生姿态异常时进入 `SYS_PAUSE`，患者姿态恢复并稳定 3 秒后自动继续。

进入 `SYS_TRAIN` 后前 500 ms 不执行姿态安全暂停检查，用于避开 PA2 手动校准刚完成时的瞬态姿态变化。`MODE_A_FIXATION` 注视稳定训练会绕开全局安全暂停，只在训练内根据头稳指标提醒和评价。

## 舵机视野标定

舵机训练范围由以下全局变量控制：

```c
uint8_t CALIB_X_MIN = 80;
uint8_t CALIB_X_MAX = 110;
uint8_t CALIB_Y_MIN = 100;
uint8_t CALIB_Y_MAX = 115;
```

语音命令“标定模式”进入 `SYS_CALIB_SERVO` 后，系统会打开激光并按顺序扫描 X/Y 轴：

1. X 轴从 70 度扫描到 150 度，按 PA2 记录一个边界。
2. X 轴从 150 度扫描回 70 度，按 PA2 记录另一个边界。
3. Y 轴从 80 度扫描到 140 度，按 PA2 记录一个边界。
4. Y 轴从 140 度扫描回 80 度，按 PA2 记录另一个边界。
5. 自动排序 min/max，更新 `CALIB_X/Y_MIN/MAX`，舵机回中并回到空闲状态。

如果某一阶段未按键，系统会使用默认兜底值，避免训练范围为空。标定值保存在 RAM 中，设备重启后恢复代码默认值。

## 设备层关键接口

| 模块 | 关键接口 | 用途 |
| --- | --- | --- |
| BMI088 | `BMI088_AsyncStart()`、`BMI088_Task()`、`BMI088_ResetReference()` | 启动异步采样、处理采样任务、重置姿态参考。 |
| SBUS | `SBUS_Init()`、`SBUS_Update()`、`SBUS_TakeSwbMiddleEvent()` | 初始化 UART5 接收、检测在线状态、消费 SWB 中位归零事件。 |
| 舵机 | `Servo_SetAngle()`、`Servo_Test()` | 控制 X/Y 轴角度和执行自检。 |
| 语音 | `Voice_Play()`、`Voice_GetCommand()` | 播放 TTS 或读取语音命令。 |
| 按键 | `Key_Scan()`、`Key_GetEvent()` | 扫描 PA2 患者按键并读取短按事件。 |
| 激光/灯光 | `Laser_On()`、`Laser_Off()`、`WS2812_Set()` | 控制训练光点和状态提示灯。 |

## UART1 VOFA 调试输出

主循环每 100 ms 通过 UART1 输出一行以 `imu:` 开头的短调试数据。USART1 当前保持 `115200, 8N1`，发送超时按本帧长度动态计算，并通过 `vofa_tx_fail_count` 记录发送失败次数。

当前字段顺序如下：

```text
imu:
euler_roll,euler_pitch,euler_yaw,
std_roll,std_pitch,std_yaw,
gyro_x,gyro_y,gyro_z,
accel_x,accel_y,accel_z,
bias_x,bias_y,bias_z,
imu_is_static,imu_calibrated,temp,
sbus_online,sbus_swa,sbus_swb,sbus_swb_middle,sbus_zero_event,
AppState,AppMode,
last_voice_cmd,last_cmd_tick,
last_tx_type,last_tx_id,last_tx_tick,
last_app_event,vofa_tx_fail_count
```

字段含义：

- 应用层欧拉角：`euler_angle.roll/pitch/yaw`。
- BMI088 内部姿态解算欧拉角：`vofa_roll_deg/pitch_deg/yaw_deg`。
- 陀螺仪、加速度计、温度和陀螺仪零偏。
- IMU 静止检测状态、校准状态。
- SBUS 在线状态、SWA/SWB 通道值、SWB 是否处于中位、归零事件标志。
- 当前应用状态 `App_GetState()` 和训练模式 `App_GetMode()`。
- 语音接收诊断：`last_voice_cmd` 和 `last_cmd_tick` 表示 MCU 最近一次收到的语音命令及其时间。
- 语音发送诊断：`last_tx_type`、`last_tx_id` 和 `last_tx_tick` 表示 MCU 最近一次发给语音模块的播报帧。
- 状态机诊断：`last_app_event` 表示最近一次应用层事件，`vofa_tx_fail_count=0` 表示 UART1 调试帧未发生发送失败。

`last_app_event` 当前事件码：

| 事件码 | 含义 |
| --- | --- |
| 0 | 无事件 |
| 1 | 进入手动姿态校准 |
| 2 | PA2 确认校准完成 |
| 3 | 重新开始当前模式 |
| 4 | 跳过当前模式 |
| 5 | 进入暂停 |
| 6 | 恢复训练 |
| 7 | 进入反馈 |
| 8 | 开始训练 |
| 9 | 进入空闲 |
| 10 | 收到“你好小盈”并进入/刷新语音监听窗口 |
| 11 | 命令收到但当前状态不处理 |

这些字段主要用于 VOFA 或串口助手观察姿态解算、SBUS 输入、语音收发和训练状态机是否正常。`AppState=4` 可能表示姿态异常暂停，也可能表示“你好小盈”语音监听暂停，需要结合 `last_voice_cmd` 和 `last_app_event` 判断。若需要排查 BMI088 DMA/中断细节，可临时恢复 `BMI088_GetDebug()` 相关字段或单独增加低频 `dbg:` 行，避免默认 `imu:` 帧过长导致串口粘连。

## 最近一次改动说明：训练状态机与语音诊断修复

本轮修复主要围绕训练交互和现场调试：

- 训练前姿态校准改为 PA2 手动确认，校准等待提示不再复用“训练暂停，请调整坐姿”。
- “重新开始”和“跳过这个”扩展到空闲、训练、暂停和反馈状态；空闲状态下也会进入校准，而不是只记录命令。
- “你好小盈”改为 5 秒静音监听窗口：训练中收到后保存现场、停止刺激、冻结计时并进入 `SYS_PAUSE`；监听窗口内训练 TTS 不再插话，超时无命令则自动恢复训练。
- 五种训练模式名支持跨状态直接切换：空闲、训练、暂停和反馈状态下都可直接进入指定模式的手动校准。
- 注视稳定训练不触发全局安全暂停，只提醒头部稳定并在结束时给稳定性反馈。
- 平稳追踪训练改为 8 个关键点追踪，到点后等待 PA2 确认，记录正确率、反应时间和头动代偿。
- 暂停恢复时补偿训练时间基准，并恢复暂停前舵机/激光状态，避免继续后立即进入反馈。
- VOFA 默认输出改为 10Hz 短帧，保留核心 IMU、SBUS、App 状态和语音诊断字段，减少 USART1 115200 下的断帧和粘连。

## 最近一次改动说明：`68edf8a imu underlying update`

最近一次提交主要更新了 IMU 底层采样和调试链路，涉及文件包括 `Core`、`Device/BMI088`、`Device/SBUS`、`Device/HeadTracker`、`.ioc` 和 Keil 工程配置。

具体变化如下：

- BMI088 驱动重构为异步采样流程，新增 DMA/中断相关状态统计，主循环通过 `BMI088_Task()` 消费采样任务。
- BMI088 姿态输出增加四元数、欧拉角、陀螺仪零偏、静止状态、校准状态和时间戳接口。
- 新增 `Device/SBUS/sbus.c` 与 `Device/SBUS/sbus.h`，使用 UART5 接收 25 字节 SBUS 帧，解析 16 个通道。
- SBUS 新增在线检测、failsafe 处理和 SWB 中位事件；SWB 进入中位时触发一次归零事件，用于 `BMI088_ResetReference()` 和 `HeadTracker_Init()`。
- 主循环扩展 VOFA 输出字段，方便同时观察 IMU、SBUS、应用状态机和 BMI088 DMA/中断调试信息。
- 头部姿态判断调整为当前垂直安装语义：`roll` 表示点头，`pitch` 表示左右转头，`yaw` 表示侧倾。
- CubeMX 和 Keil 工程同步了 UART5、DMA、中断和新增源码文件配置。

## 编译与提交建议

使用 Keil MDK 打开以下工程文件编译：

```text
MDK-ARM/Eyes_Rehabilitation_Device.uvprojx
```

提交到 Git 时，建议优先提交源码和工程配置：

```powershell
git add App Core Device Eyes_Rehabilitation_Device.ioc MDK-ARM/Eyes_Rehabilitation_Device.uvprojx MDK-ARM/Eyes_Rehabilitation_Device.uvoptx MDK-ARM/RTE README.md
git commit -m "docs: add firmware readme"
```

谨慎提交 Keil 编译产物，例如：

```text
*.o
*.d
*.axf
*.map
*.hex
*.build_log.htm
```

这些文件通常由本地编译生成，体积较大，且容易因为重新编译产生无关差异。除非团队明确要求保存固件产物，否则推荐只提交源码、配置文件和文档。
