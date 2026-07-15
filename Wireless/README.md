# 无线手持控制器说明

本目录包含眼功能康复训练设备的无线手持控制器。系统由三块主控组成：

```text
ESP32-S3-Touch-LCD-5
        |
        | ESP-NOW，固定信道 1
        v
XIAO ESP32C3
        |
        | USART10，115200 8N1
        v
STM32H723 / DC-MC02
```

屏幕负责训练计划、参数、本地报告和状态显示；XIAO只负责ESP-NOW与串口帧的双向转发；STM32是训练状态机、舵机、激光、BMI088、语音、安全逻辑和原始报告的唯一执行主体。

## 工程目录

| 路径 | 用途 |
| --- | --- |
| `Handheld_S3/Handheld_S3.ino` | 5寸触摸屏中文训练控制界面 |
| `Xiao_C3_Bridge/Xiao_C3_Bridge.ino` | ESP-NOW与USART10桥接 |
| `MacAddress/MacAddress.ino` | 查询ESP32的Wi-Fi STA MAC地址 |
| `Handheld_S3/protocol.h` | 屏幕端44字节通信协议 |
| `Xiao_C3_Bridge/protocol.h` | XIAO端44字节通信协议 |
| `Handheld_S3/ui_font_20.c` | Noto Sans SC 20像素精简中文字库 |

## Arduino环境

- Arduino IDE 2.x
- `ESP32 by Espressif Systems 3.0.7`
- `esp-lib-utils 0.1.2`
- `ESP32_Display_Panel 1.0.0`
- `ESP32_IO_Expander 1.0.1`
- `LVGL 8.4.0`

不要升级到LVGL 9，当前微雪移植层按LVGL 8.4.0编写。

当前 `lv_conf.h` 设置 `LV_USE_FONT_COMPRESSED=0`，因此重新生成 `ui_font_20.c` 时必须给 `lv_font_conv` 添加 `--no-compress --no-prefilter --lv-include lvgl.h`，并确认生成文件中的 `bitmap_format = 0`。当前字库包含界面实际使用的237个中文字符；压缩字库可以通过编译，但会导致屏幕上的中文、英文和数字全部不显示。

### 屏幕开发板设置

```text
Board                 ESP32S3 Dev Module
USB Mode              Hardware CDC and JTAG
USB CDC On Boot       Enabled
Flash Size            16MB
PSRAM                 OPI PSRAM
Partition Scheme      16M Flash (3MB APP/9.9MB FATFS)
Upload Mode           UART0 / Hardware CDC
```

### XIAO开发板设置

```text
Board                 XIAO_ESP32C3
Upload Speed          921600
```

上传不稳定时将XIAO的Upload Speed降到115200。

## MAC配对

当前固定地址为：

```text
屏幕 STA MAC    44:1B:F6:97:F6:B8
XIAO STA MAC    AC:27:6E:7E:10:6C
ESP-NOW信道     1
加密            关闭
```

更换模块后，分别烧录 `MacAddress.ino` 获取新地址，然后更新：

- 屏幕 `XIAO_PEER_MAC`
- XIAO `HANDHELD_PEER_MAC`

## USART10接线

```text
STM32 PE3 / USART10_TX  -> XIAO D7 / GPIO20 / RX
STM32 PE2 / USART10_RX  <- XIAO D6 / GPIO21 / TX
STM32 GND                -  XIAO GND
```

两端均使用 `115200 8N1`。TX必须接对方RX，两块板必须共地。

初次联调时让STM32和XIAO分别通过各自接口供电，不连接UART10的VCC。最终使用DC-MC02的UART10插座供电时，官方原理图确认该VCC为5V：

```text
DC-MC02 VCC(5V) -> SS14/1N5819无条纹端
二极管有条纹端 -> XIAO 5V
DC-MC02 GND      -> XIAO GND
```

不要把5V接到XIAO的3V3引脚。直接连接外部5V时不要同时插XIAO USB；串联肖特基二极管可以降低USB倒灌风险，插拔线路仍应先断电。

## 通信协议

所有链路统一传输44字节 `EyeFrame`：

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| magic | 2 | 固定 `0x5945` |
| version | 1 | 当前版本4 |
| type | 1 | 消息类型 |
| sequence | 4 | 命令序号与去重依据 |
| payload_length | 2 | 有效载荷长度，最大32 |
| payload | 32 | 固定载荷区 |
| crc16 | 2 | CRC16-CCITT |

消息类型：

| 类型 | 值 | 方向 | 用途 |
| --- | ---: | --- | --- |
| `PING/PONG` | 1/2 | 屏幕与XIAO双向 | 无线测试 |
| `UART_TEST/UART_ACK` | 3/4 | 屏幕与STM32双向 | 完整链路测试 |
| `STATUS` | 5 | 双向 | 链路心跳 |
| `CONTROL` | 6 | 屏幕到STM32 | 开始、暂停、继续、停止、标定 |
| `CONTROL_ACK` | 7 | STM32到屏幕 | 命令执行结果 |
| `TRAINING_STATUS` | 8 | STM32到屏幕 | 每200ms训练状态 |
| `CONFIG_SET/ACK` | 9/10 | 屏幕与STM32双向 | 训练参数设置与确认 |
| `REPORT_QUERY/DATA` | 11/12 | 屏幕与STM32双向 | 分段查询训练报告 |
| `CONFIG_GET/DATA` | 13/14 | 屏幕与STM32双向 | 读取当前舵机范围和训练参数 |
| `SERVO_PREVIEW/ACK` | 15/16 | 屏幕与STM32双向 | 空闲状态实时预览舵机位置 |

控制结果包括：接受、重复、状态不允许、模式无效、队列忙、字段无效、范围越界、顺序无效和版本不匹配。屏幕等待ACK超时会使用相同序号重发，最多3次；STM32识别重复序号后只回复，不会重复执行训练命令。

训练状态使用整数传输，包含系统状态、完整/自选流程、模式进度、PA2等待标志、正确/超时次数、平均反应时间、0.1度头部偏移/波动、标定阶段、标定点数、训练时长和会话编号。头部偏移由 `sqrt(pitch^2 + roll^2)` 计算，不使用容易漂移的yaw。

配置载荷固定32字节，覆盖X/Y训练范围与初始位置、舵机稳定时间、五种模式剂量/速度/路径以及受保护的安全参数。舵机固定硬范围为 `10°～170°`，必须满足 `min < home < max`，X/Y跨度至少为 `10°/5°`。配置只允许在空闲或模式选择状态应用。

报告使用屏幕主动查询：先读取会话摘要，再逐模式读取基础指标和左右侧/代偿细节。XIAO使用高优先级队列、发送回调和最近分段缓存，失败最多重试3次；屏幕独立排队报告帧并只重试当前分段。

## 烧录顺序

1. Arduino IDE打开并上传 `Xiao_C3_Bridge.ino`。
2. Arduino IDE打开并上传 `Handheld_S3.ino`。
3. Keil打开 `MDK-ARM/Eyes_Rehabilitation_Device.uvprojx`，编译并下载STM32固件。
4. 断电完成USART10接线，再依次上电。

## 操作流程

1. “参数设置”页选择预设或逐项调整，点击“保存并同步”。安全参数需长按解锁，且只能比固件硬上限更严格。
2. “训练控制”页选择完整训练，或选择自选训练后依次点击1至5个模式；按钮数字表示执行顺序。
3. 若激光位置偏向一侧，点击“标定模式”，按PA2开始扫描，并在X/Y轴四次经过有效视野边界时分别按PA2记录。
4. 安装调试时可在参数页长按“舵机测试”，断开患者佩戴并确认激光关闭后，以 `-1°/+1°` 调整位置，设置最小/初始/最大并保存；取消会回到 `90°/90°`。
5. 点击“开始训练”。若参数尚未同步，屏幕会先完成配置ACK，再自动发送开始命令。
6. STM32进入姿态校准，患者调整姿态后按PA2开始；多项训练之间仍由PA2确认进入下一项。
7. “暂停”只恢复人工暂停；姿态异常安全暂停必须满足稳定时间后自动恢复。“停止”立即关闭刺激并返回空闲。
8. 会话完成后屏幕自动读取报告；“训练报告”页可浏览最近20次记录并手动刷新。

语音、实体按键和屏幕控制可以同时使用。PA2仍负责姿态校准、患者反馈和完整模式的下一项确认。

## 联调与排查

- “连接测试”页的无线测试只检查屏幕与XIAO。
- 完整链路测试经过屏幕、XIAO、USART10、STM32后原路返回。
- `BAD`/错误计数应保持0，正常完整往返延迟通常为几十毫秒。
- 无线在线、主控离线：检查PE2/PE3是否交叉连接、是否共地、两端是否都是115200 8N1。
- 两端都离线：检查固定MAC、ESP-NOW信道和XIAO供电。
- 屏幕黑屏：确认16MB Flash、OPI PSRAM和正确的屏幕板型设置。
- 页面内容叠加：确认使用当前整帧刷新版本；启动会清空双帧缓存，页签切换和报告更新会触发全屏失效重绘，FFat/Preferences写入不再占用LVGL锁。
- 中文显示方框：确认 `ui_font_20.c` 与屏幕工程在同一目录且成功参与编译。
- 参数被拒绝：检查角度是否在 `10°～170°`、大小关系、最小跨度，或当前是否为空闲状态。
- 报告超时：查看连接测试页“报告查询/串口返回/无线成功/无线失败”。查询不增说明屏幕到XIAO失败；有查询无返回说明USART10或STM32处理失败；有返回且无线失败增加说明ESP-NOW投递失败。
- 显示“固件版本不匹配”：STM32、XIAO和屏幕必须同时烧录协议版本4、配置版本2固件。

## 当前边界

- 本地报告最多保留20次，尚未连接云服务器、小程序或用户账号。
- 当前硬件没有眼动相机；平稳追踪报告反映按键完成和头部代偿，不等同于眼球轨迹测量。
- 当前视觉聚焦只有一个固定距离激光目标，报告评估的是目标确认和按键反应，不代表眼调焦能力。
- 训练表现评分用于训练过程比较，不作为医疗诊断或疗效结论。
