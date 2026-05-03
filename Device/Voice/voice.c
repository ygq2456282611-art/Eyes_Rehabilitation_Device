#include "voice.h"
#include "usart.h"

static UART_HandleTypeDef *huart = NULL;

void Voice_Init(void)
{
    huart = &huart1;
}

void Voice_SendFrame(uint8_t type, uint8_t id)
{
    if (huart == NULL) return;
    uint8_t frame[5] = {0xAA, 0x55, type, id, 0xFB};
    HAL_UART_Transmit(huart, frame, 5, 100);
}

void Voice_Play(uint8_t type, uint8_t id)
{
    Voice_SendFrame(type, id);
}

uint8_t Voice_GetCommand(void)
{
    return 0;
}
