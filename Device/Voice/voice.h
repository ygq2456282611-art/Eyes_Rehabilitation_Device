#ifndef VOICE_H
#define VOICE_H

#include "main.h"

/* 
 * 亚博 AI 语音交互模块 (CI1302) UART 驱动
 * 固件：CI1302_中文_单麦_UART1_115200_2M.bin
 * 波特率：115200, 8N1
 * 协议：AA 55 [TYPE] [ID] FB
 *   - TYPE=0xFF  → 播报命令（MCU 主动触发 TTS）
 *   - TYPE=0x00  → 语音命令（MCU 触发命令词，模块播报关联内容）
 */

/* ===== 命令词 ID（与 Excel 协议列表严格对应） ===== */

/* 语音命令 (CMD=0x00, 发送 AA 55 00 XX FB) */
#define VOICE_CMD_CALIB_STILL   0x01  /* 校准静态 */
#define VOICE_CMD_FIXATION      0x02  /* 注视稳定训练 */
#define VOICE_CMD_SACCADE       0x03  /* 扫视训练 */
#define VOICE_CMD_PURSUIT       0x04  /* 平稳追踪训练 */
#define VOICE_CMD_FOCUS         0x05  /* 视觉聚焦训练 */
#define VOICE_CMD_NEGLECT       0x06  /* 空间忽略训练 */

/* 播报命令 (CMD=0xFF, 发送 AA 55 FF XX FB) */
#define VOICE_TTS_CALIB_DONE    0x3D  /* 校准完成 */
#define VOICE_TTS_KEEP_STILL    0x3E  /* 请保持头部稳定 */
#define VOICE_TTS_CORRECT       0x3F  /* 正确 / 答得很棒 */
#define VOICE_TTS_TIMEOUT       0x40  /* 超时 */
#define VOICE_TTS_MISS          0x42  /* 错过，请注意该位置 */
#define VOICE_TTS_EYE_ONLY      0x43  /* 请用眼睛转，不要转头 */
#define VOICE_TTS_FIND_LIGHT    0x44  /* 请寻找光点 */
#define VOICE_TTS_NEGLECT_HINT  0x49  /* 超时，请注意该侧空间 */
#define VOICE_TTS_INIT_OK       0x58  /* 初始化完成 / 开始识别 */
#define VOICE_TTS_TRAIN_DONE    0x5E  /* 训练结束，太棒了 */
#define VOICE_TTS_WELCOME       0x5F  /* 欢迎 */

/* 函数声明 */
void Voice_Init(void);
void Voice_Play(uint8_t type, uint8_t id);
void Voice_SendFrame(uint8_t type, uint8_t id);
uint8_t Voice_GetCommand(void);

#endif
