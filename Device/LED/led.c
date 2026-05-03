/**
 * @file    led.c
 * @brief   LED 指示驱动模块
 *          2 颗 LED：聚焦训练指示(PB8)、状态指示(PB9)
 *          支持常亮、常灭、翻转和闪烁模式
 */
#include "led.h"

static GPIO_TypeDef *led_ports[2] = {GPIOB, GPIOB};
static uint16_t      led_pins[2]  = {GPIO_PIN_8, GPIO_PIN_9};

/**
 * @brief  初始化所有 LED
 *         默认全部熄灭
 */
void LED_Init(void)
{
    LED_Off(LED_FOCUS);
    LED_Off(LED_STATUS);
}

/**
 * @brief  打开指定 LED
 * @param  id : LED_FOCUS=0（聚焦）, LED_STATUS=1（状态）
 */
void LED_On(uint8_t id)
{
    if (id > LED_STATUS) return;
    HAL_GPIO_WritePin(led_ports[id], led_pins[id], GPIO_PIN_SET);
}

/**
 * @brief  关闭指定 LED
 * @param  id : LED 编号
 */
void LED_Off(uint8_t id)
{
    if (id > LED_STATUS) return;
    HAL_GPIO_WritePin(led_ports[id], led_pins[id], GPIO_PIN_RESET);
}

/**
 * @brief  翻转指定 LED 的亮灭状态
 * @param  id : LED 编号
 */
void LED_Toggle(uint8_t id)
{
    if (id > LED_STATUS) return;
    HAL_GPIO_TogglePin(led_ports[id], led_pins[id]);
}

/**
 * @brief  LED 闪烁（非阻塞式）
 * @param  id        : LED 编号
 * @param  period_ms : 闪烁周期（ms），最小 100ms
 *         基于 HAL_GetTick() 时间戳翻转状态，不阻塞主循环
 */
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
