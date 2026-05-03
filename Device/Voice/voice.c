/**
 * @file    voice.c
 * @brief   语音合成模块驱动
 *          通过 USART1 发送帧协议数据驱动语音模块播报
 *          帧格式：FD + 长度 + 命令 + 数据
 *          支持播放文本、预设提示音、音量调节、停止
 */
#include "voice.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *voice_uart = NULL;

/**
 * @brief  初始化语音模块
 *         绑定 USART1 句柄
 */
void Voice_Init(void)
{
    voice_uart = &huart1;
}

/**
 * @brief  播报文本内容
 * @param  text : 要播报的中文文本字符串（GBK编码）
 *         构造帧格式：FD + 长度(3+文本长度) + 命令(01合成) + 编码(01) + 文本 + 终止(00)
 */
void Voice_PlayText(const char *text)
{
    if (voice_uart == NULL || text == NULL) return;

    char frame[256];
    uint8_t len = (uint8_t)strlen(text);
    if (len > 240) len = 240;

    frame[0] = 0xFD;
    frame[1] = len + 3;
    frame[2] = 0x00;
    frame[3] = 0x01;
    frame[4] = 0x01;

    memcpy(&frame[5], text, len);
    frame[5 + len] = 0x00;

    HAL_UART_Transmit(voice_uart, (uint8_t *)frame, (uint16_t)(6 + len), 1000);
}

/**
 * @brief  播放预设提示音
 * @param  id : 提示音编号（0~，由语音模块预定义）
 *         命令 0x02 表示播放预设提示音
 */
void Voice_PlayPrompt(uint8_t id)
{
    if (voice_uart == NULL) return;

    uint8_t frame[] = {0xFD, 0x03, 0x00, 0x02, id};
    HAL_UART_Transmit(voice_uart, frame, 5, 500);
}

/**
 * @brief  设置音量
 * @param  vol : 音量 0~18（由模块支持的级别）
 *         命令 0x03 表示音量设置
 */
void Voice_SetVolume(uint8_t vol)
{
    if (voice_uart == NULL) return;
    if (vol > 18) vol = 18;

    uint8_t frame[] = {0xFD, 0x03, 0x00, 0x03, vol};
    HAL_UART_Transmit(voice_uart, frame, 5, 500);
}

/**
 * @brief  停止当前播报
 *         命令 0x04 表示停止播放
 */
void Voice_Stop(void)
{
    if (voice_uart == NULL) return;

    uint8_t frame[] = {0xFD, 0x02, 0x00, 0x04};
    HAL_UART_Transmit(voice_uart, frame, 4, 500);
}

/**
 * @brief  查询语音模块是否忙碌
 * @return 0=空闲, 1=忙碌（当前简化为始终返回0）
 */
uint8_t Voice_IsBusy(void)
{
    return 0;
}
