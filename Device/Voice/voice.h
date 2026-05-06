/**
 * @file    voice.h
 * @brief   亚博 AI 语音交互模块 (CI1302) 驱动 —— 公共接口与命令词定义
 *          基于 UART（115200, 8N1）与模块通信
 *          模块固件：CI1302_中文_单麦_UART1_115200_2M.bin
 *
 * @section 通信协议
 *           帧格式：AA 55 [TYPE] [ID] FB
 *           - AA 55     : 帧头（2 字节固定）
 *           - TYPE      : 命令类型（0xFF=播报, 0x00=语音命令）
 *           - ID        : 命令词编号（对应 Excel 协议列表）
 *           - FB        : 帧尾
 *
 *           发送（MCU → 模块）：
 *             播报 TTS → AA 55 FF [ID] FB  → 模块立即播报对应文本
 *             语音命令 → AA 55 00 [ID] FB  → 模块播报关联内容 + 等待语音识别
 *
 *           接收（模块 → MCU，语音识别结果）：
 *             AA 55 [TYPE] [ID] FB → 通过 UART RX 中断接收
 *             上层调用 Voice_GetCommand() 获取 ID
 *
 * @section 接线
 *           PA9  (USART1_TX) → 模块 RX
 *           PA10 (USART1_RX) → 模块 TX（可选，用于接收识别结果）
 *           VCC 3.3V → 模块 VCC
 *           GND     → 模块 GND
 */
#ifndef VOICE_H
#define VOICE_H

#include "main.h"

/* ========================================================================
 * 命令词 ID 定义
 * 用户需在亚博平台配置固件时，将以下 ID 与对应播报文本绑定
 * ======================================================================== */

/* -------- 语音命令（TYPE=0x00，发送 AA 55 00 XX FB） -------- */
/* 这些是"命令词"，通常通过语音说出口令触发，同时模块播报反馈内容 */
#define VOICE_CMD_CALIB_STILL   0x01  /* "校准静态" — 进入校准模式 */
#define VOICE_CMD_FIXATION      0x02  /* "注视稳定训练" */
#define VOICE_CMD_SACCADE       0x03  /* "扫视训练" */
#define VOICE_CMD_PURSUIT       0x04  /* "平稳追踪训练" */
#define VOICE_CMD_FOCUS         0x05  /* "视觉聚焦训练" */
#define VOICE_CMD_NEGLECT       0x06  /* "空间忽略训练" */

/* -------- 播报命令（TYPE=0xFF，发送 AA 55 FF XX FB） -------- */
/* 这些是"纯播报"命令，MCU 主动触发 TTS，模块只需播放不需等待语音 */
#define VOICE_TTS_CALIB_DONE    0x3D  /* "校准完成" */
#define VOICE_TTS_KEEP_STILL    0x3E  /* "请保持头部稳定" — 注视训练时提醒 */
#define VOICE_TTS_CORRECT       0x3F  /* "答得很棒" — 扫视正确响应 */
#define VOICE_TTS_TIMEOUT       0x40  /* "时间到咯，再试一次" — 超时提示 */
#define VOICE_TTS_MISS          0x42  /* "错过，请注意该位置" */
#define VOICE_TTS_EYE_ONLY      0x43  /* "请用眼睛转，不要转头" — 代偿警告 */
#define VOICE_TTS_FIND_LIGHT    0x44  /* "请寻找光点" — 忽略训练提示 */
#define VOICE_TTS_NEGLECT_HINT  0x49  /* "超时，请注意该侧空间" */
#define VOICE_TTS_INIT_OK       0x58  /* "初始化完成" — 系统上电播报 */
#define VOICE_TTS_TRAIN_DONE    0x5E  /* "训练结束，太棒了" */
#define VOICE_TTS_WELCOME       0x5F  /* "欢迎使用眼部康复装置" */

/* ========================================================================
 * 函数声明
 * ======================================================================== */

/**
 * @brief  初始化语音模块
 *         绑定 USART1 句柄，启动 UART RX 中断（单字节接收模式）
 */
void Voice_Init(void);

/**
 * @brief  发送播报/命令帧到语音模块（主动播报）
 * @param  type : 命令类型（0xFF=播报TTS, 0x00=语音命令）
 * @param  id   : 命令词 ID（见本文件宏定义）
 *         示例：
 *           Voice_Play(0xFF, VOICE_TTS_INIT_OK);   // 播报"初始化完成"
 *           Voice_Play(0x00, VOICE_CMD_FIXATION);  // 播报"注视稳定训练"+等待语音
 */
void Voice_Play(uint8_t type, uint8_t id);

/**
 * @brief  发送 5 字节原始帧（AA 55 type id FB）
 * @param  type : 帧第 3 字节
 * @param  id   : 帧第 4 字节
 *         由 Voice_Play 内部调用，也可直接使用
 */
void Voice_SendFrame(uint8_t type, uint8_t id);

/**
 * @brief  获取语音识别结果（非阻塞）
 * @return 0 = 无新命令；非 0 = 识别到的命令词 ID
 *         轮询调用，每帧只返回一次，消费后清零
 */
uint8_t Voice_GetCommand(void);

/**
 * @brief  获取 UART 句柄（供中断回调比较使用）
 * @return huart1 的指针
 */
UART_HandleTypeDef* Voice_GetUART(void);

#endif
