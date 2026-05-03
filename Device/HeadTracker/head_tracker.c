/**
 * @file    head_tracker.c
 * @brief   头部位姿分析与安全检测模块
 *          基于 BMI088 卡尔曼滤波输出的欧拉角，实时分析患者头部：
 *          - 头部姿势识别（正中/左/右/上/下）
 *          - 头部稳定度计算（角度标准差）
 *          - 代偿性转头检测（偏盲患者常见代偿行为）
 *          - 安全性检测（异常倾斜/抖动）
 */
#include "head_tracker.h"
#include <math.h>
#include <string.h>

static HeadAnalysis_t result;

/* ===== 阈值常量（可根据实际测试调整） ===== */
static const float COMPENSATORY_YAW_THRESHOLD  = 15.0f;  /* 偏航角超过15°视为代偿性转头 */
static const float TREMOR_STD_THRESHOLD        = 3.0f;   /* 头稳指标超过3°视为抖动 */
static const float TILT_THRESHOLD              = 30.0f;  /* 俯仰/横滚超过30°触发安全警报 */

/* ===== 状态保持变量 ===== */
static float prev_pitch = 0, prev_roll = 0, prev_yaw = 0;
static float pitch_buffer[100], roll_buffer[100];  /* 循环缓冲区，存储最近100帧角度 */
static uint8_t buffer_idx = 0, buffer_full = 0;
static float elapsed = 0;
static uint8_t tremor_counter = 0;      /* 抖动持续计数 */
static uint32_t last_update_ms = 0;
static float last_pitch_valid = 0, last_roll_valid = 0;

/**
 * @brief  初始化头部分析器
 *         清零所有缓存和状态变量
 */
void HeadTracker_Init(void)
{
    memset(&result, 0, sizeof(HeadAnalysis_t));
    memset(pitch_buffer, 0, sizeof(pitch_buffer));
    memset(roll_buffer, 0, sizeof(roll_buffer));
    buffer_idx = 0; buffer_full = 0;
    elapsed = 0;
    prev_pitch = 0; prev_roll = 0; prev_yaw = 0;
    last_update_ms = 0;
    last_pitch_valid = 0; last_roll_valid = 0;
}

/**
 * @brief  更新头部分析结果（需每10ms调用一次）
 * @param  euler : BMI088 输出的欧拉角数据指针
 * @param  dt    : 时间步长（固定 0.01s）
 *         分析流程：
 *         1) 计算角度增量（用于检测瞬时运动）
 *         2) 判断代偿性转头：|yaw| > 15°
 *         3) 判断安全倾斜：|pitch|>30° 或 |roll|>30°
 *         4) 循环缓冲区更新，计算角度标准差作为头稳指标
 *         5) 抖动检测：标准差连续10帧超阈值
 *         6) 姿势分类：根据 pitch/roll 相对大小判断抬头/低头/左偏/右偏
 */
void HeadTracker_Update(bmi088_euler_data_t *euler, float dt)
{
    uint32_t now = HAL_GetTick();
    float dt_ms = (float)(now - last_update_ms);
    last_update_ms = now;

    /* 1) 计算角度变化量 */
    result.pitch_delta = euler->pitch - prev_pitch;
    result.roll_delta  = euler->roll  - prev_roll;
    result.yaw_delta   = euler->yaw   - prev_yaw;

    prev_pitch = euler->pitch;
    prev_roll  = euler->roll;
    prev_yaw   = euler->yaw;

    /* 2) 代偿性转头检测：偏盲患者常通过转头代替眼球运动 */
    float abs_yaw = fabsf(euler->yaw);
    if (abs_yaw > COMPENSATORY_YAW_THRESHOLD)
    {
        result.is_compensatory = 1;
    }
    else
    {
        result.is_compensatory = 0;
    }

    /* 3) 安全倾斜检测：异常躯干倾斜 */
    if (fabsf(euler->pitch) > TILT_THRESHOLD || fabsf(euler->roll) > TILT_THRESHOLD)
    {
        result.safety_alert = 1;
    }
    else
    {
        result.safety_alert = 0;
    }

    /* 4) 循环缓存 — 计算头稳指标（角度标准差） */
    pitch_buffer[buffer_idx] = euler->pitch;
    roll_buffer[buffer_idx]  = euler->roll;
    buffer_idx++;
    if (buffer_idx >= 100) { buffer_idx = 0; buffer_full = 1; }

    uint8_t sample_count = buffer_full ? 100 : buffer_idx;
    if (sample_count > 0)
    {
        float p_mean = 0, r_mean = 0;
        for (uint8_t i = 0; i < sample_count; i++)
        {
            p_mean += pitch_buffer[i];
            r_mean += roll_buffer[i];
        }
        p_mean /= sample_count;
        r_mean /= sample_count;

        float p_var = 0, r_var = 0;
        for (uint8_t i = 0; i < sample_count; i++)
        {
            float dp = pitch_buffer[i] - p_mean;
            float dr = roll_buffer[i] - r_mean;
            p_var += dp * dp;
            r_var += dr * dr;
        }
        p_var /= sample_count;
        r_var /= sample_count;

        result.head_stability = sqrtf(p_var + r_var);

        last_pitch_valid = p_mean;
        last_roll_valid = r_mean;
    }

    /* 5) 抖动检测：头稳指标持续超阈值判定为头抖 */
    if (result.head_stability > TREMOR_STD_THRESHOLD)
    {
        tremor_counter++;
        if (tremor_counter > 10)
        {
            result.is_tremor = 1;
        }
    }
    else
    {
        tremor_counter = 0;
        result.is_tremor = 0;
    }

    /* 6) 姿势分类 */
    float abs_pitch = fabsf(result.pitch);
    float abs_roll  = fabsf(result.roll);

    if (abs_pitch > 20.0f && abs_pitch > abs_roll)
    {
        if (result.pitch > 0) result.posture = POSTURE_UP;
        else                  result.posture = POSTURE_DOWN;
    }
    else if (abs_roll > 15.0f && abs_roll > abs_pitch)
    {
        if (result.roll > 0) result.posture = POSTURE_RIGHT;
        else                 result.posture = POSTURE_LEFT;
    }
    else
    {
        result.posture = POSTURE_CENTER;
    }

    result.pitch = euler->pitch;
    result.roll  = euler->roll;
    result.yaw   = euler->yaw;
}

/**
 * @brief  获取当前头部姿态分析结果
 * @return HeadAnalysis_t 结构体指针，包含所有分析指标
 */
HeadAnalysis_t* HeadTracker_GetResult(void)
{
    return &result;
}

/**
 * @brief  综合安全检查（异常倾斜 或 抖动）
 * @return 0=安全, 1=存在风险（触发暂停训练）
 */
uint8_t HeadTracker_CheckSafety(void)
{
    return result.safety_alert | result.is_tremor;
}
