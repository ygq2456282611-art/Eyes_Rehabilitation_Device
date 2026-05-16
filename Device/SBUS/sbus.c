#include "sbus.h"

#include "usart.h"

#include <string.h>

#define SBUS_FRAME_LEN 25U
#define SBUS_HEADER 0x0FU
#define SBUS_ONLINE_TIMEOUT_MS 300U
#define SBUS_SWITCH_MIDDLE_MIN 700U
#define SBUS_SWITCH_MIDDLE_MAX 1300U
#define SBUS_ZERO_EVENT_COOLDOWN_MS 500U

static uint8_t sbus_rx_buf[SBUS_FRAME_LEN];
static volatile uint16_t sbus_channels[SBUS_CHANNEL_COUNT];
static volatile uint8_t sbus_online;
static volatile uint8_t sbus_failsafe;
static volatile uint8_t sbus_lost_frame;
static volatile uint8_t sbus_swb_was_middle;
static volatile uint8_t sbus_swb_middle_event;
static volatile uint32_t sbus_last_frame_ms;
static volatile uint32_t sbus_last_zero_event_ms;

static void SBUS_StartReceive(void);
static void SBUS_ParseFrame(const uint8_t *buf);
static sbus_switch_pos_t SBUS_SwitchPositionFromValue(uint16_t value);

void SBUS_Init(void)
{
    memset(sbus_rx_buf, 0, sizeof(sbus_rx_buf));
    memset((void *)sbus_channels, 0, sizeof(sbus_channels));
    sbus_online = 0U;
    sbus_failsafe = 0U;
    sbus_lost_frame = 0U;
    sbus_swb_was_middle = 0U;
    sbus_swb_middle_event = 0U;
    sbus_last_frame_ms = 0U;
    sbus_last_zero_event_ms = 0U;
    SBUS_StartReceive();
}

void SBUS_Update(uint32_t now_ms)
{
    if ((sbus_online != 0U) && ((now_ms - sbus_last_frame_ms) > SBUS_ONLINE_TIMEOUT_MS))
    {
        sbus_online = 0U;
        sbus_swb_was_middle = 0U;
    }
}

uint8_t SBUS_IsOnline(void)
{
    return sbus_online;
}

uint16_t SBUS_GetChannel(uint8_t ch_index)
{
    if (ch_index >= SBUS_CHANNEL_COUNT)
    {
        return 0U;
    }

    return sbus_channels[ch_index];
}

sbus_switch_pos_t SBUS_GetSwbPosition(void)
{
    return SBUS_SwitchPositionFromValue(sbus_channels[SBUS_SWB_CH_INDEX]);
}

uint8_t SBUS_IsSwbMiddle(void)
{
    return (uint8_t)(SBUS_GetSwbPosition() == SBUS_SWITCH_MIDDLE);
}

uint8_t SBUS_TakeSwbMiddleEvent(void)
{
    uint8_t event = sbus_swb_middle_event;
    sbus_swb_middle_event = 0U;
    return event;
}

static void SBUS_StartReceive(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_FRAME_LEN);
    if (huart5.hdmarx != 0)
    {
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}

static void SBUS_ParseFrame(const uint8_t *buf)
{
    uint8_t swb_is_middle = 0U;
    uint32_t now_ms = HAL_GetTick();

    if (buf[0] != SBUS_HEADER)
    {
        return;
    }

    sbus_channels[0]  = (uint16_t)(((uint16_t)buf[1]       | ((uint16_t)buf[2] << 8)) & 0x07FFU);
    sbus_channels[1]  = (uint16_t)((((uint16_t)buf[2] >> 3) | ((uint16_t)buf[3] << 5)) & 0x07FFU);
    sbus_channels[2]  = (uint16_t)((((uint16_t)buf[3] >> 6) | ((uint16_t)buf[4] << 2) | ((uint16_t)buf[5] << 10)) & 0x07FFU);
    sbus_channels[3]  = (uint16_t)((((uint16_t)buf[5] >> 1) | ((uint16_t)buf[6] << 7)) & 0x07FFU);
    sbus_channels[4]  = (uint16_t)((((uint16_t)buf[6] >> 4) | ((uint16_t)buf[7] << 4)) & 0x07FFU);
    sbus_channels[5]  = (uint16_t)((((uint16_t)buf[7] >> 7) | ((uint16_t)buf[8] << 1) | ((uint16_t)buf[9] << 9)) & 0x07FFU);
    sbus_channels[6]  = (uint16_t)((((uint16_t)buf[9] >> 2) | ((uint16_t)buf[10] << 6)) & 0x07FFU);
    sbus_channels[7]  = (uint16_t)((((uint16_t)buf[10] >> 5) | ((uint16_t)buf[11] << 3)) & 0x07FFU);
    sbus_channels[8]  = (uint16_t)(((uint16_t)buf[12]      | ((uint16_t)buf[13] << 8)) & 0x07FFU);
    sbus_channels[9]  = (uint16_t)((((uint16_t)buf[13] >> 3) | ((uint16_t)buf[14] << 5)) & 0x07FFU);
    sbus_channels[10] = (uint16_t)((((uint16_t)buf[14] >> 6) | ((uint16_t)buf[15] << 2) | ((uint16_t)buf[16] << 10)) & 0x07FFU);
    sbus_channels[11] = (uint16_t)((((uint16_t)buf[16] >> 1) | ((uint16_t)buf[17] << 7)) & 0x07FFU);
    sbus_channels[12] = (uint16_t)((((uint16_t)buf[17] >> 4) | ((uint16_t)buf[18] << 4)) & 0x07FFU);
    sbus_channels[13] = (uint16_t)((((uint16_t)buf[18] >> 7) | ((uint16_t)buf[19] << 1) | ((uint16_t)buf[20] << 9)) & 0x07FFU);
    sbus_channels[14] = (uint16_t)((((uint16_t)buf[20] >> 2) | ((uint16_t)buf[21] << 6)) & 0x07FFU);
    sbus_channels[15] = (uint16_t)((((uint16_t)buf[21] >> 5) | ((uint16_t)buf[22] << 3)) & 0x07FFU);

    sbus_lost_frame = (uint8_t)((buf[23] & 0x04U) != 0U);
    sbus_failsafe = (uint8_t)((buf[23] & 0x08U) != 0U);
    sbus_online = (uint8_t)(sbus_failsafe == 0U);
    sbus_last_frame_ms = now_ms;

    swb_is_middle = SBUS_IsSwbMiddle();
    if ((swb_is_middle != 0U) && (sbus_swb_was_middle == 0U) &&
        ((now_ms - sbus_last_zero_event_ms) >= SBUS_ZERO_EVENT_COOLDOWN_MS))
    {
        sbus_swb_middle_event = 1U;
        sbus_last_zero_event_ms = now_ms;
    }

    sbus_swb_was_middle = swb_is_middle;
}

static sbus_switch_pos_t SBUS_SwitchPositionFromValue(uint16_t value)
{
    if (value < SBUS_SWITCH_MIDDLE_MIN)
    {
        return SBUS_SWITCH_LOW;
    }

    if (value <= SBUS_SWITCH_MIDDLE_MAX)
    {
        return SBUS_SWITCH_MIDDLE;
    }

    return SBUS_SWITCH_HIGH;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        if (Size == SBUS_FRAME_LEN)
        {
            SBUS_ParseFrame(sbus_rx_buf);
        }

        SBUS_StartReceive();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        sbus_online = 0U;
        sbus_swb_was_middle = 0U;
        SBUS_StartReceive();
    }
}
