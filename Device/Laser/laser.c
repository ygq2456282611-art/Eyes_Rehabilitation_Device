#include "laser.h"

static uint8_t laser_state = 0;

void Laser_Init(void)
{
    laser_state = 0;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void Laser_On(void)
{
    laser_state = 1;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
}

void Laser_Off(void)
{
    laser_state = 0;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

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
