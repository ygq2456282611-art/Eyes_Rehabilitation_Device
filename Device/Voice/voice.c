/**
 * @file    voice.c
 * @brief   亚博 AI 语音交互模块 (CI1302) 驱动实现
 *          基于 UART7（PE7=RX, PE8=TX, 115200, 8N1）与 CI1302 芯片的双向通信：
 *          - TX：发送 5 字节帧触发 TTS 播报或语音命令
 *          - RX：中断接收模块返回的识别结果帧
 *
 * @note UART RX 中断工作方式：
 *       HAL_UART_Receive_IT(&huart7, buf, 1) → 每收到 1 字节触发
 *       HAL_UART_RxCpltCallback() → 逐字节拼装帧 → 校验帧尾 0xFB → 提取 ID
 */
#include "voice.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* UART7 句柄指针，由 Voice_Init 赋值 */
static UART_HandleTypeDef *huart = NULL;

/* 重定向 printf 到 USART1（调试输出） */
int fputc(int ch, FILE *f)
{
    if (huart == NULL) return ch;
    HAL_UART_Transmit(huart, (uint8_t *)&ch, 1, 100);
    return ch;
}

/* ============ UART RX 帧接收状态 ============ */
#define RX_BUF_SIZE  16                     /* 接收缓冲区大小 */
static uint8_t  rx_buf[RX_BUF_SIZE];        /* 字节接收缓冲区 */
static uint8_t  rx_idx = 0;                 /* 当前接收位置 */
static uint8_t  rx_frame_ready = 0;         /* 完整帧就绪标志 */
static uint8_t  rx_frame_type = 0;          /* 最新帧的 TYPE */
static uint8_t  rx_frame_id   = 0;          /* 最新帧的 ID */

/**
 * @brief  初始化语音模块
 *         绑定额外的 USART1，启动单字节 RX 中断接收
 *         RX 中断会由 HAL_UART_RxCpltCallback 自动处理
 */
void Voice_Init(void)
{
    huart = &huart7;
    rx_idx = 0;
    rx_frame_ready = 0;
    HAL_Delay(200);
    HAL_UART_Receive_IT(huart, rx_buf, 1);
}

/**
 * @brief  UART 接收完成回调（HAL 弱函数，此处覆盖）
 * @param  huart : 触发中断的 UART 句柄
 * @note   每收到 1 字节进入一次，进行帧同步和校验
 *
 *         帧同步逻辑：
 *         - 字节 0 期望 0xAA，否则丢弃
 *         - 字节 1 期望 0x55，否则复位并检查是否为新帧的 0xAA
 *         - 收集 5 字节后校验帧尾 0xFB，通过则提取 TYPE 和 ID
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 只处理本模块的 UART */
    if (huart != Voice_GetUART()) return;

    uint8_t byte = rx_buf[0];

    /* 帧头同步：等待 0xAA */
    if (rx_idx == 0 && byte != 0xAA)
    {
        HAL_UART_Receive_IT(huart, rx_buf, 1);
        return;
    }

    /* 帧头同步：需要 0x55 跟在 0xAA 之后 */
    if (rx_idx == 1 && byte != 0x55)
    {
        rx_idx = 0;
        /* 如果当前字节是 0xAA，可能是一个新帧的开始 */
        if (byte == 0xAA) rx_buf[rx_idx++] = byte;
        HAL_UART_Receive_IT(huart, rx_buf, 1);
        return;
    }

    /* 收集字节到缓冲区 */
    rx_buf[rx_idx++] = byte;

    /* 满 5 字节 → 校验帧尾并提取有效数据 */
    if (rx_idx >= 5)
    {
        /* 校验帧尾 0xFB */
        if (rx_buf[4] == 0xFB)
        {
            rx_frame_type = rx_buf[2];
            rx_frame_id   = rx_buf[3];
            rx_frame_ready = 1;
        }
        rx_idx = 0;  /* 复位，准备下一帧 */
    }

    /* 继续等待下一字节 */
    HAL_UART_Receive_IT(huart, rx_buf, 1);
}

/**
 * @brief  发送 5 字节帧（AA 55 TYPE ID FB）
 * @param  type : 帧 TYPE 字节
 * @param  id   : 帧 ID 字节
 *         通过 USART1 阻塞发送，超时 100ms
 */
void Voice_SendFrame(uint8_t type, uint8_t id)
{
    if (huart == NULL) return;
    uint8_t frame[5] = {0xAA, 0x55, type, id, 0xFB};
    HAL_UART_Transmit(huart, frame, 5, 100);
}

/**
 * @brief  向语音模块发送播报/命令（对 Voice_SendFrame 的封装）
 * @param  type : 0xFF=播报TTS, 0x00=语音命令
 * @param  id   : 命令词 ID
 */
void Voice_Play(uint8_t type, uint8_t id)
{
    Voice_SendFrame(type, id);
}

/**
 * @brief  获取最新识别的语音命令 ID
 * @return 0 = 无新命令；非 0 = 命令词 ID
 *         每帧只返回一次，调用后自动清除就绪标志
 */
uint8_t Voice_GetCommand(void)
{
    if (rx_frame_ready)
    {
        rx_frame_ready = 0;
        return rx_frame_id;
    }
    return 0;
}

/**
 * @brief  返回 UART 句柄指针（供中断回调中比较使用）
 * @return huart1 的地址
 */
UART_HandleTypeDef* Voice_GetUART(void)
{
    return huart;
}
