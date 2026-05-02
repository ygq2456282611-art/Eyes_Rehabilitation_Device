#include "servo.h"

static TIM_HandleTypeDef *servo_tim[2] = {NULL, NULL};
static uint32_t servo_ch[2] = {TIM_CHANNEL_3, TIM_CHANNEL_1};

void Servo_Init(void)
{
    servo_tim[SERVO_AXIS_X] = &htim1;
    servo_tim[SERVO_AXIS_Y] = &htim1;

    HAL_TIM_PWM_Start(servo_tim[SERVO_AXIS_X], servo_ch[SERVO_AXIS_X]);
    HAL_TIM_PWM_Start(servo_tim[SERVO_AXIS_Y], servo_ch[SERVO_AXIS_Y]);

    Servo_SetAngle(SERVO_AXIS_X, 90);
    Servo_SetAngle(SERVO_AXIS_Y, 90);
}

void Servo_SetAngle(uint8_t axis, uint8_t angle)
{
    if (axis > SERVO_AXIS_Y || servo_tim[axis] == NULL) return;
    if (angle > 180) angle = 180;

    uint16_t pulse = SERVO_PULSE_MIN + (uint32_t)angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN) / 180;
    __HAL_TIM_SET_COMPARE(servo_tim[axis], servo_ch[axis], pulse);
}

void Servo_SetUs(uint8_t axis, uint16_t us)
{
    if (axis > SERVO_AXIS_Y || servo_tim[axis] == NULL) return;
    if (us < SERVO_PULSE_MIN) us = SERVO_PULSE_MIN;
    if (us > SERVO_PULSE_MAX) us = SERVO_PULSE_MAX;

    __HAL_TIM_SET_COMPARE(servo_tim[axis], servo_ch[axis], us);
}

void Servo_SmoothMove(uint8_t axis, uint8_t target_angle, uint16_t time_ms)
{
    if (axis > SERVO_AXIS_Y || servo_tim[axis] == NULL) return;
    if (target_angle > 180) target_angle = 180;
    if (time_ms < 10) time_ms = 10;

    uint32_t current_ccr = __HAL_TIM_GET_COMPARE(servo_tim[axis], servo_ch[axis]);
    uint8_t current_angle = (uint8_t)((uint32_t)(current_ccr - SERVO_PULSE_MIN) * 180 / (SERVO_PULSE_MAX - SERVO_PULSE_MIN));

    int16_t step = (int16_t)target_angle - (int16_t)current_angle;
    uint16_t steps = time_ms / 20;
    if (steps < 1) steps = 1;

    float angle_per_step = (float)step / steps;

    for (uint16_t i = 1; i <= steps; i++)
    {
        uint8_t angle = (uint8_t)(current_angle + angle_per_step * i);
        Servo_SetAngle(axis, angle);
        HAL_Delay(20);
    }
    Servo_SetAngle(axis, target_angle);
}

void Servo_Test(void)
{
    Servo_SetAngle(SERVO_AXIS_X, 0);  Servo_SetAngle(SERVO_AXIS_Y, 0);  HAL_Delay(500);
    Servo_SetAngle(SERVO_AXIS_X, 45); Servo_SetAngle(SERVO_AXIS_Y, 45); HAL_Delay(500);
    Servo_SetAngle(SERVO_AXIS_X, 90); Servo_SetAngle(SERVO_AXIS_Y, 90); HAL_Delay(500);
    Servo_SetAngle(SERVO_AXIS_X, 135);Servo_SetAngle(SERVO_AXIS_Y, 135);HAL_Delay(500);
    Servo_SetAngle(SERVO_AXIS_X, 180);Servo_SetAngle(SERVO_AXIS_Y, 180);HAL_Delay(500);
    Servo_SetAngle(SERVO_AXIS_X, 90); Servo_SetAngle(SERVO_AXIS_Y, 90);
}
