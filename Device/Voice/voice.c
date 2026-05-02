#include "voice.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *voice_uart = NULL;

void Voice_Init(void)
{
    voice_uart = &huart1;
}

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

void Voice_PlayPrompt(uint8_t id)
{
    if (voice_uart == NULL) return;

    uint8_t frame[] = {0xFD, 0x03, 0x00, 0x02, id};
    HAL_UART_Transmit(voice_uart, frame, 5, 500);
}

void Voice_SetVolume(uint8_t vol)
{
    if (voice_uart == NULL) return;
    if (vol > 18) vol = 18;

    uint8_t frame[] = {0xFD, 0x03, 0x00, 0x03, vol};
    HAL_UART_Transmit(voice_uart, frame, 5, 500);
}

void Voice_Stop(void)
{
    if (voice_uart == NULL) return;

    uint8_t frame[] = {0xFD, 0x02, 0x00, 0x04};
    HAL_UART_Transmit(voice_uart, frame, 4, 500);
}

uint8_t Voice_IsBusy(void)
{
    return 0;
}
