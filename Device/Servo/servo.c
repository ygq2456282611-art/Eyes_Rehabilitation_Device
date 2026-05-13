/**
 * @file    servo.c
 * @brief   舵机云台驱动模块
 *          基于 TIM1 的 PWM 输出，驱动双轴舵机（X轴俯仰、Y轴偏航）
 *          TIM1 配置：Prescaler=240, Period=19999 → 50Hz (20ms)
 *          脉宽范围：500~2500us，对应 0°~180°
 */
#include "servo.h"

/* 两个舵机共用一个 TIM1 句柄，不同通道 */
static TIM_HandleTypeDef *servo_tim[2] = {NULL, NULL};
static uint32_t servo_ch[2] = {TIM_CHANNEL_3, TIM_CHANNEL_1};

/**
 * @brief  初始化舵机
 *         绑定 TIM1 句柄，启动 PWM 输出，复位到 90° 居中位置
 */
void Servo_Init(void)
{
    servo_tim[SERVO_AXIS_X] = &htim1;
    servo_tim[SERVO_AXIS_Y] = &htim1;

    HAL_TIM_PWM_Start(servo_tim[SERVO_AXIS_X], servo_ch[SERVO_AXIS_X]);
    HAL_TIM_PWM_Start(servo_tim[SERVO_AXIS_Y], servo_ch[SERVO_AXIS_Y]);

    Servo_SetAngle(SERVO_AXIS_X, 90);
    Servo_SetAngle(SERVO_AXIS_Y, 90);
}

/**
 * @brief  设置舵机角度
 * @param  axis  : 轴编号（SERVO_AXIS_X=0俯仰, SERVO_AXIS_Y=1偏航）
 * @param  angle : 目标角度 0~180°
 *         内部将角度线性映射到脉宽：500 + (angle/180)*2000 us
 */
void Servo_SetAngle(uint8_t axis, uint8_t angle)
{
    if (axis > SERVO_AXIS_Y || servo_tim[axis] == NULL) return;
    if (angle > 180) angle = 180;

    uint16_t pulse = SERVO_PULSE_MIN + (uint32_t)angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN) / 180;
    __HAL_TIM_SET_COMPARE(servo_tim[axis], servo_ch[axis], pulse);
}

/**
 * @brief  直接设置舵机脉宽（微秒）
 * @param  axis : 轴编号
 * @param  us   : 脉宽 500~2500us
 *         用于更精细的舵机位置控制
 */
void Servo_SetUs(uint8_t axis, uint16_t us)
{
    if (axis > SERVO_AXIS_Y || servo_tim[axis] == NULL) return;
    if (us < SERVO_PULSE_MIN) us = SERVO_PULSE_MIN;
    if (us > SERVO_PULSE_MAX) us = SERVO_PULSE_MAX;

    __HAL_TIM_SET_COMPARE(servo_tim[axis], servo_ch[axis], us);
}

/**
 * @brief  平滑移动到目标角度（分段插值）
 * @param  axis       : 轴编号
 * @param  target_angle : 目标角度 0~180°
 * @param  time_ms    : 移动总时长（ms），每 20ms 插一步
 *         用于平稳追踪训练中的连续轨迹模拟
 */
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

/**
 * @brief  舵机自检函数（分轴测试，防止云台机械碰撞）
 *         先 X 轴(俯仰)独立走 0°→45°→90°→135°→180°→复位
 *         再 Y 轴(偏航)独立走同样流程
 */
void Servo_Test(void)
{
    /* X轴测试：Y轴保持居中 */
    Servo_SetAngle(SERVO_AXIS_Y, 90);
    Servo_SetAngle(SERVO_AXIS_X, 0);   HAL_Delay(500);
    Servo_SetAngle(SERVO_AXIS_X, 45);  HAL_Delay(500);
    Servo_SetAngle(SERVO_AXIS_X, 90);  HAL_Delay(500);
    Servo_SetAngle(SERVO_AXIS_X, 135); HAL_Delay(500);
    Servo_SetAngle(SERVO_AXIS_X, 90);  HAL_Delay(300);

    /* Y轴测试：X轴保持居中 */
    Servo_SetAngle(SERVO_AXIS_Y, 90);  HAL_Delay(1000);
    Servo_SetAngle(SERVO_AXIS_Y, 125);  HAL_Delay(1000);
    Servo_SetAngle(SERVO_AXIS_Y, 150);  HAL_Delay(1000);
    Servo_SetAngle(SERVO_AXIS_Y, 90);
}
