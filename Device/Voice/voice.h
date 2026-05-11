/**
 * @file    voice.h
 * @brief   亚博 AI 语音交互模块 (CI1302) 驱动 —— 公共接口与命令词定义
 *          基于 UART（115200, 8N1）与模块通信
 *
 * @section 通信协议
 *           帧格式：AA 55 [TYPE] [ID] FB
 *           - TYPE=0x01/0x02 : 唤醒词
 *           - TYPE=0x00      : 语音命令（MCU触发，模块播报关联内容）
 *           - TYPE=0xFF      : 播报TTS（MCU触发纯文本播报）
 *
 * @section 接线
 *           PA9  (USART1_TX) → 模块 RX
 *           PA10 (USART1_RX) → 模块 TX（可选，用于接收识别结果）
 */
#ifndef VOICE_H
#define VOICE_H

#include "main.h"

/* ========================================================================
 * 命令词 ID 定义（对应 2026-05-11 新固件协议）
 * 固件通信协议：
 *   <welcome>    : AA 55 01 00 FB
 *   <inactivate> : AA 55 02 6F FB
 *   你好小幻     : AA 55 03 00 FB
 *   ...
 *   关闭播报     : AA 55 0A 00 FB
 * ======================================================================== */

/* -------- 语音命令（TYPE=0x00，发送 AA 55 00 XX FB） -------- */
#define VOICE_CMD_CALIB_STILL   0x01  /* 校准静态 */
#define VOICE_CMD_FIXATION      0x02  /* 注视稳定训练 */
#define VOICE_CMD_SACCADE       0x03  /* 扫视训练 */
#define VOICE_CMD_PURSUIT       0x04  /* 平稳追踪训练 */
#define VOICE_CMD_FOCUS         0x05  /* 视觉聚焦训练 */
#define VOICE_CMD_NEGLECT       0x06  /* 空间忽略训练 */

/* -------- 播报命令（TYPE=0xFF，发送 AA 55 FF XX FB） -------- */
#define VOICE_TTS_CALIB_DONE    0x01  /* 校准完成 */
#define VOICE_TTS_KEEP_STILL    0x02  /* 请保持头部稳定 */
#define VOICE_TTS_CORRECT       0x03  /* 正确 / 答得很棒 */
#define VOICE_TTS_TIMEOUT       0x04  /* 超时 */
#define VOICE_TTS_MISS          0x05  /* 错过，请注意该位置 */
#define VOICE_TTS_EYE_ONLY      0x06  /* 姿态监测异常 / 请用眼睛转 */
#define VOICE_TTS_FIND_LIGHT    0x07  /* 请寻找光点 */
#define VOICE_TTS_NEGLECT_HINT  0x08  /* 空间忽略提示 */
#define VOICE_TTS_INIT_OK       0x09  /* 初始化完成 */
#define VOICE_TTS_TRAIN_DONE    0x0A  /* 训练结束 */
#define VOICE_TTS_WELCOME       0x0B  /* 欢迎 */


/* ========================================================================
 * 函数声明
 * ======================================================================== */
void Voice_Init(void);
void Voice_Play(uint8_t type, uint8_t id);
void Voice_SendFrame(uint8_t type, uint8_t id);
uint8_t Voice_GetCommand(void);
UART_HandleTypeDef* Voice_GetUART(void);

#endif
