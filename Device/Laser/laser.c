/**
 * @file    laser.c
 * @brief   激光控制驱动模块
 *          通过 PA0 GPIO 控制单路激光二极管的开关和闪烁
 */
#include "laser.h"

static uint8_t laser_state = 0;

/**
 * @brief  初始化激光
 *         默认关闭激光（PA0 输出低电平）
 */
void Laser_Init(void)
{
    laser_state = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

/**
 * @brief  打开激光
 *         PA0 输出高电平，激光点亮
 */
void Laser_On(void)
{
    laser_state = 1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

/**
 * @brief  关闭激光
 *         PA0 输出低电平，激光熄灭
 */
void Laser_Off(void)
{
    laser_state = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

/**
 * @brief  激光闪烁
 * @param  period_ms : 闪烁周期（ms），最小 50ms
 * @param  count     : 闪烁次数
 *         用于训练开始/结束时的视觉提示
 */
void Laser_Blink(uint16_t period_ms, uint8_t count)
{
    if (period_ms < 50) period_ms = 50;
    uint16_t half = period_ms / 2;

    for (uint8_t i = 0; i < count; i++)
    {
        Laser_On();
        HAL_Delay(half);
        Laser_Off();
        HAL_Delay(half);
    }
}

/**
 * @brief  激光自检：闪烁 3 次验证 PA0 输出
 */
void Laser_Test(void)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        Laser_On();
        HAL_Delay(500);
        Laser_Off();
        HAL_Delay(500);
    }
}
