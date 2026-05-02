#include "key.h"

#define KEY_COUNT         3
#define KEY_DEBOUNCE_MS   30
#define KEY_LONG_PRESS_MS 800

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

void Key_Init(void)
{
    keys[KEY_MODE].port    = GPIOD;
    keys[KEY_MODE].pin     = GPIO_PIN_15;
    keys[KEY_CONFIRM].port = GPIOD;
    keys[KEY_CONFIRM].pin  = GPIO_PIN_14;
    keys[KEY_BACK].port    = GPIOE;
    keys[KEY_BACK].pin     = GPIO_PIN_14;

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
