/**
 * @file    ws2812.c
 * @brief   板载 WS2812 RGB LED 驱动实现
 *          PA7 (SPI6_MOSI) 通过 SPI6 模拟 WS2812 单总线协议
 *
 *          SPI6 时钟 = HSE(24MHz), prescaler=4 → 6MHz
 *          每数据 bit 用 2 个 SPI 时钟编码：
 *          0→0x60（0110_0000）, 1→0x78（0111_1000）
 *
 * @note    严格匹配官方例程 CtrBoard-H7_WS2812 的 SPI 参数
 */
#include "ws2812.h"
#include <math.h>

static SPI_HandleTypeDef hspi6_ws;
static uint8_t ws2812_ready = 0;
static uint16_t breathe_step = 0;

void WS2812_Init(void)
{
    /* 1) 配置 SPI6 时钟源为 HSE（24MHz） */
    RCC_PeriphCLKInitTypeDef clk = {0};
    clk.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    clk.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE;
    HAL_RCCEx_PeriphCLKConfig(&clk);

    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 2) 配置 PA5(SCK) + PA7(MOSI) 为 SPI6 复用 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* 3) 配置 SPI6：主机、仅发送、8bit、CPOL=Low、CPHA=2Edge、6MHz */
    hspi6_ws.Instance = SPI6;
    hspi6_ws.Init.Mode = SPI_MODE_MASTER;
    hspi6_ws.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    hspi6_ws.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi6_ws.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi6_ws.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi6_ws.Init.NSS = SPI_NSS_SOFT;
    hspi6_ws.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi6_ws.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi6_ws.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi6_ws.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi6_ws.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    HAL_SPI_Init(&hspi6_ws);

    ws2812_ready = 1;
    WS2812_Set(0, 0, 0);
}

void WS2812_Set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!ws2812_ready) return;

    uint8_t txbuf[24];
    uint8_t zero = 0;

    for (int i = 0; i < 8; i++)
    {
        txbuf[7 - i]  = (((g >> i) & 0x01) ? 0x78 : 0x60);
        txbuf[15 - i] = (((r >> i) & 0x01) ? 0x78 : 0x60);
        txbuf[23 - i] = (((b >> i) & 0x01) ? 0x78 : 0x60);
    }

    HAL_SPI_Transmit(&hspi6_ws, txbuf, 24, 100);
    for (int i = 0; i < 100; i++)
        HAL_SPI_Transmit(&hspi6_ws, &zero, 1, 100);
}

void WS2812_BreathingGreen(void)
{
    if (!ws2812_ready) return;

    float rad = (float)breathe_step * 3.14159f * 2.0f / 100.0f;
    uint8_t brightness = (uint8_t)((sinf(rad - 1.5708f) + 1.0f) * 127.5f);

    WS2812_Set(0, brightness, 0);

    breathe_step++;
    if (breathe_step >= 100) breathe_step = 0;
}
