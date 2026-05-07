/**
 * @file    buzzer.c
 * @brief   板载蜂鸣器驱动实现（TIM12_CH2, PB15）
 *          参考官方例程 CtrBoard-H7_BUZZER 实现
 *          定时器 TIM12 时钟 = APB1 × 2 = 240MHz
 *          Prescaler = 24-1 → 10MHz
 *          Period = 2000-1 → 5kHz PWM
 *          CCR2 = 50~1000 → 占空比可调
 */
#include "buzzer.h"

static TIM_HandleTypeDef htim12_buzzer;
static uint8_t is_init = 0;

void Buzzer_Init(void)
{
    if (is_init) return;

    /* 1) 使能 TIM12 时钟 */
    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 2) 配置 PB15 为 TIM12_CH2 复用功能 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM12;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 3) 配置 TIM12 PWM */
    htim12_buzzer.Instance = TIM12;
    htim12_buzzer.Init.Prescaler = 24 - 1;
    htim12_buzzer.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim12_buzzer.Init.Period = 2000 - 1;
    htim12_buzzer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim12_buzzer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim12_buzzer);

    /* 4) 配置 PWM 通道 */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim12_buzzer, &oc, TIM_CHANNEL_2);

    /* 5) 初始状态：关闭 */
    HAL_TIM_PWM_Start(&htim12_buzzer, TIM_CHANNEL_2);
    TIM12->CCR2 = 0;

    is_init = 1;
}

void Buzzer_On(uint16_t volume)
{
    if (!is_init) return;
    TIM12->CCR2 = (volume > 1000) ? 1000 : volume;
}

void Buzzer_Off(void)
{
    if (!is_init) return;
    TIM12->CCR2 = 0;
}

void Buzzer_Alert(uint8_t count, uint16_t on_ms, uint16_t off_ms)
{
    if (!is_init) return;
    for (uint8_t i = 0; i < count; i++)
    {
        Buzzer_On(500);          /* 50% 占空比，最响 */
        HAL_Delay(on_ms);
        Buzzer_Off();
        if (i < count - 1)
            HAL_Delay(off_ms);
    }
}
