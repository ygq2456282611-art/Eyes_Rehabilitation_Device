#include "led.h"

static GPIO_TypeDef *led_ports[2] = {GPIOB, GPIOB};
static uint16_t      led_pins[2]  = {GPIO_PIN_8, GPIO_PIN_9};

void LED_Init(void)
{
    LED_Off(LED_FOCUS);
    LED_Off(LED_STATUS);
}

void LED_On(uint8_t id)
{
    if (id > LED_STATUS) return;
    HAL_GPIO_WritePin(led_ports[id], led_pins[id], GPIO_PIN_SET);
}

void LED_Off(uint8_t id)
{
    if (id > LED_STATUS) return;
    HAL_GPIO_WritePin(led_ports[id], led_pins[id], GPIO_PIN_RESET);
}

void LED_Toggle(uint8_t id)
{
    if (id > LED_STATUS) return;
    HAL_GPIO_TogglePin(led_ports[id], led_pins[id]);
}

void LED_Blink(uint8_t id, uint16_t period_ms)
{
    if (id > LED_STATUS) return;
    if (period_ms < 100) period_ms = 100;

    uint32_t now = HAL_GetTick();
    static uint32_t last_toggle[2] = {0, 0};
    static uint8_t  state[2] = {0, 0};

    if (now - last_toggle[id] >= period_ms)
    {
        last_toggle[id] = now;
        state[id] = !state[id];
        if (state[id])
            LED_On(id);
        else
            LED_Off(id);
    }
}
