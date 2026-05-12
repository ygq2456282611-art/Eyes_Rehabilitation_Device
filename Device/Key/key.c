/**
 * @file    key.c
 * @brief   按键驱动模块（软件消抖 + 长短按检测）
 *          4 个按键：模式选择(PD15)、确认(PD14)、返回(PE14)、患者反馈(PA2)
 *          采用轮询方式（Key_Scan 需 10ms 周期调用）
 *          检测逻辑：低电平按下 → 消抖 30ms → 短按 or 长按 800ms
 */
#include "key.h"

#define KEY_COUNT         4
#define KEY_DEBOUNCE_MS   30
#define KEY_LONG_PRESS_MS 800

/**
 * @brief  按键对象结构体
 * @param  port        : GPIO 端口
 * @param  pin         : GPIO 引脚号
 * @param  state       : 当前电平状态（0=松开, 1=按下）
 * @param  last_state  : 上一次电平状态（用于检测边沿变化）
 * @param  press_tick  : 按下瞬间的时间戳
 * @param  release_tick: 松开瞬间的时间戳
 * @param  event       : 检测到的事件类型
 * @param  event_ready : 事件就绪标志（上层读取后清除）
 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       state;
    uint8_t       last_state;
    uint32_t      press_tick;
    uint32_t      release_tick;
    uint8_t       event;
    uint8_t       event_ready;
} KeyObj_t;

static KeyObj_t keys[KEY_COUNT];

/**
 * @brief  初始化按键模块
 *         绑定 GPIO 端口/引脚，清除所有状态
 */
void Key_Init(void)
{
    keys[KEY_MODE].port    = GPIOD;
    keys[KEY_MODE].pin     = GPIO_PIN_15;
    keys[KEY_CONFIRM].port = GPIOD;
    keys[KEY_CONFIRM].pin  = GPIO_PIN_14;
    keys[KEY_BACK].port    = GPIOE;
    keys[KEY_BACK].pin     = GPIO_PIN_14;
    keys[KEY_PATIENT].port = GPIOA;
    keys[KEY_PATIENT].pin  = GPIO_PIN_2;

    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        keys[i].state       = 0;
        keys[i].last_state  = 0;
        keys[i].press_tick  = 0;
        keys[i].release_tick= 0;
        keys[i].event       = KEY_EVENT_NONE;
        keys[i].event_ready = 0;
    }
}

/**
 * @brief  按键扫描（需 10ms 周期调用）
 *         检测流程：
 *         1) 读取当前引脚电平（低电平=按下）
 *         2) 边沿检测：按下时记录 press_tick，松开时判断消抖并产生短按事件
 *         3) 持续按下超过 800ms 产生长按事件
 *         事件由 Key_GetEvent() 消费
 */
void Key_Scan(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        keys[i].state = (HAL_GPIO_ReadPin(keys[i].port, keys[i].pin) == GPIO_PIN_RESET) ? 1 : 0;

        if (keys[i].state != keys[i].last_state)
        {
            if (keys[i].state == 1)
            {
                keys[i].press_tick = now;
            }
            else
            {
                keys[i].release_tick = now;
                if (now - keys[i].press_tick >= KEY_DEBOUNCE_MS)
                {
                    keys[i].event = KEY_EVENT_SHORT;
                    keys[i].event_ready = 1;
                }
            }
            keys[i].last_state = keys[i].state;
        }

        if (keys[i].state == 1 && keys[i].event_ready == 0)
        {
            if (now - keys[i].press_tick >= KEY_LONG_PRESS_MS)
            {
                keys[i].event = KEY_EVENT_LONG;
                keys[i].event_ready = 1;
            }
        }
    }
}

/**
 * @brief  获取按键事件
 * @param  key_id : 按键编号（KEY_MODE=0, KEY_CONFIRM=1, KEY_BACK=2）
 * @return 事件类型：NONE/SHORT/LONG
 *         每次调用只返回一次，消费后事件清除
 */
KeyEvent_t Key_GetEvent(uint8_t key_id)
{
    if (key_id >= KEY_COUNT) return KEY_EVENT_NONE;

    if (keys[key_id].event_ready)
    {
        keys[key_id].event_ready = 0;
        return (KeyEvent_t)keys[key_id].event;
    }
    return KEY_EVENT_NONE;
}
