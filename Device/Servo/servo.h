#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#include "tim.h"

#define SERVO_PULSE_MIN  500
#define SERVO_PULSE_MAX  2500
#define SERVO_PULSE_MID  1500
#define SERVO_ANGLE_MIN  0
#define SERVO_ANGLE_MAX  180

#define SERVO_AXIS_X  0
#define SERVO_AXIS_Y  1

void Servo_Init(void);
void Servo_SetAngle(uint8_t axis, uint8_t angle);
void Servo_SetUs(uint8_t axis, uint16_t us);
void Servo_SmoothMove(uint8_t axis, uint8_t target_angle, uint16_t time_ms);
void Servo_Test(void);

#endif
