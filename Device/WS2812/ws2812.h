/**
 * @file    ws2812.h
 * @brief   板载 WS2812 RGB LED 驱动（PA7, SPI6_MOSI）
 *          参考 DM-02 官方例程 CtrBoard-H7_WS2812
 */
#ifndef WS2812_H
#define WS2812_H

#include "main.h"

void WS2812_Init(void);
void WS2812_Set(uint8_t r, uint8_t g, uint8_t b);
void WS2812_BreathingGreen(void);

#endif
