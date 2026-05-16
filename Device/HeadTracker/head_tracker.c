/**
 * @file    head_tracker.c
 * @brief   头部位姿分析与安全检测模块
 *          基于 BMI088 卡尔曼滤波输出的欧拉角，实时分析患者头部：
 *          - 头部姿势识别（正中/左/右/上/下）
 *          - 头部稳定度计算（pitch/roll 角度标准差）
 *          - 代偿性转头检测（偏盲患者常见代偿行为）
 *          - 安全性检测（异常前倾/后倾/抖动）
 * @note    坐标系：板子 USB 口朝上佩戴，BMI088 的 X 轴朝上
 *          → pitch = 前后倾（点头/仰头）
 *          → roll  = 左右转头
 *          → yaw   = 侧倾（耳朵靠肩，因无磁力计不准，仅作参考）
 */
#include "head_tracker.h"
#include <math.h>
#include <string.h>

static HeadAnalysis_t result;
static uint8_t last_pitch_valid = 0;
static uint8_t last_roll_valid = 0;
static uint32_t last_update_ms = 0;

/* ============ 可调阈值 ============ */
static const float COMPENSATORY_ROLL_THRESHOLD  = 20.0f;  /* roll 超过此值视为代偿转头 */
static const float TREMOR_STD_THRESHOLD         = 3.0f;   /* 头稳标准差超过此值视为头抖 */
static const float TILT_THRESHOLD               = 30.0f;  /* pitch 超过此值触发安全警报 */

/* ============ 内部状态 ============ */
static float prev_pitch = 0, prev_roll = 0, prev_yaw = 0;
static float pitch_buffer[100];  /* 最近 100 帧 pitch 缓存（用于计算标准差） */
static float roll_buffer[100];   /* 最近 100 帧 roll 缓存 */
static uint8_t buffer_idx = 0;
static uint8_t buffer_full = 0;
static uint8_t tremor_counter = 0;

/**
 * @brief  初始化头部分析器
 *         清零所有内部状态、缓存和输出结果
 */
void HeadTracker_Init(void)
{
    memset(&result, 0, sizeof(HeadAnalysis_t));
    memset(pitch_buffer, 0, sizeof(pitch_buffer));
    memset(roll_buffer, 0, sizeof(roll_buffer));
    buffer_idx = 0; buffer_full = 0;
    prev_pitch = 0; prev_roll = 0; prev_yaw = 0;
    last_pitch_valid = 0; last_roll_valid = 0;
}

/**
 * @brief  更新头部分析结果（每 10ms 调用一次）
 * @param  euler : BMI088 输出的欧拉角指针
 * @param  dt    : 时间步长（固定 0.01s）
 *
 *         分析流程（按执行顺序）：
 *         1. 计算角度增量（瞬时运动速度）
 *         2. 立即更新 result.pitch/roll/yaw（供后续分析使用）
 *         3. 代偿性转头检测：|roll| > 20° 说明患者用转代替替了眼球运动
 *         4. 安全倾斜检测：|pitch| > 30° 说明躯干姿态异常
 *         5. 循环缓冲区更新，计算标准差作为头部稳定度指标
 *         6. 抖动检测：标准差连续 10 帧超阈值判定为病理性头抖
 *         7. 姿势分类：根据 pitch/roll 的绝对值大小判断低头/仰头/左偏/右偏
 */
void HeadTracker_Update(bmi088_euler_data_t *euler, float dt)
{
    (void)dt;
    uint32_t now = HAL_GetTick();
    last_update_ms = now;

    /* 1) 角度增量 */
    result.pitch_delta = euler->pitch - prev_pitch;
    result.roll_delta  = euler->roll  - prev_roll;
    result.yaw_delta   = euler->yaw   - prev_yaw;

    prev_pitch = euler->pitch;
    prev_roll  = euler->roll;
    prev_yaw   = euler->yaw;

    /* 2) 立即更新输出值（姿势分类前必须赋值） */
    result.pitch = euler->pitch;
    result.roll  = euler->roll;
    result.yaw   = euler->yaw;

    /* 3) 代偿性转头检测
     *    偏盲患者常用转头代替眼球扫视。用 roll 检测左右转头。
     *    若 |roll| > 20°，标记为代偿行为，训练模式据此提示 "用眼睛看" */
    float abs_pitch = fabsf(result.pitch);
    result.is_compensatory = (abs_pitch > COMPENSATORY_ROLL_THRESHOLD) ? 1 : 0;

    /* 4) 安全倾斜检测
     *    pitch 反映前后倾。若 |pitch| > 30° 说明患者前倾或后仰过度，
     *    可能影响训练效果或带来跌倒风险，触发暂停 */
    result.safety_alert = (fabsf(result.roll) > TILT_THRESHOLD) ? 1 : 0;

    /* 5) 循环缓存 + 头稳指标（标准差）
     *    将当前 pitch/roll 存入环形缓冲区，计算最近 N 帧的联合标准差。
     *    头稳指标 = sqrt(var(pitch) + var(roll))，越小说明头部越稳定 */
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
    }

    /* 6) 抖动检测
     *    头稳指标连续 10 帧超过 3° 判定为病理性头抖，
     *    触发安全暂停，防止因抖动导致训练无效 */
    if (result.head_stability > TREMOR_STD_THRESHOLD)
    {
        tremor_counter++;
        if (tremor_counter > 10)
            result.is_tremor = 1;
    }
    else
    {
        tremor_counter = 0;
        result.is_tremor = 0;
    }

    /* 7) 姿势分类
     *    规则：
     *    - pitch 绝对值 > 20° 且比 roll 更突出 → 抬头(pitch>0)/低头(pitch<0)
     *    - roll 绝对值 > 15° 且比 pitch 更突出 → 右偏(roll>0)/左偏(roll<0)
     *    - 否则 → 正中 */
    abs_pitch = fabsf(result.pitch);
    float abs_roll = fabsf(result.roll);

    if (abs_roll > 20.0f && abs_roll > abs_pitch)
    {
        result.posture = (result.roll > 0) ? POSTURE_UP : POSTURE_DOWN;
    }
    else if (abs_pitch > 15.0f && abs_pitch > abs_roll)
    {
        result.posture = (result.pitch > 0) ? POSTURE_RIGHT : POSTURE_LEFT;
    }
    else
    {
        result.posture = POSTURE_CENTER;
    }
}

/**
 * @brief  获取当前头部分析结果
 * @return HeadAnalysis_t 结构体指针，包含所有分析指标
 */
HeadAnalysis_t* HeadTracker_GetResult(void)
{
    return &result;
}

/**
 * @brief  综合安全检查
 * @return 0=安全, 非0=存在风险
 *         - bit0: 异常倾斜
 *         - bit1: 头抖
 *         训练状态机根据此值决定是否暂停训练
 */
uint8_t HeadTracker_CheckSafety(void)
{
    return result.safety_alert | result.is_tremor;
}
