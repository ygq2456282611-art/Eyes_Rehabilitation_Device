#ifndef LED_H
#define LED_H

#include "main.h"

#define LED_FOCUS   0
#define LED_STATUS  1

void LED_Init(void);
void LED_On(uint8_t id);
void LED_Off(uint8_t id);
void LED_Toggle(uint8_t id);
void LED_Blink(uint8_t id, uint16_t period_ms);

#endif
