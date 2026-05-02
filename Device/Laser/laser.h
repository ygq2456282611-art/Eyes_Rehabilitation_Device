#ifndef LASER_H
#define LASER_H

#include "main.h"

void Laser_Init(void);
void Laser_On(void);
void Laser_Off(void);
void Laser_Blink(uint16_t period_ms, uint8_t count);

#endif
