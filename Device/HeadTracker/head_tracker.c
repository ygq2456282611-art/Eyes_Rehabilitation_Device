/**
 * @file    head_tracker.c
 * @brief   头部位姿分析与安全检测模块
 *          基于 BMI088 卡尔曼滤波输出的欧拉角，实时分析患者头部：
 *          - 头部姿势识别（正中/左/右/上/下）
 *          - 头部稳定度计算（pitch/roll 角度标准差）
 *          - 代偿性转头检测（偏盲患者常见代偿行为）
 *          - 安全性检测（异常前倾/后倾/抖动）
 *
 * @note    坐标系适配（板子 USB 口朝上佩戴）：
 *          BMI088 芯片 X 轴朝 USB 口（上方），Y 轴朝 XT30 口（左侧）
 *          → pitch = 绕 Y 轴旋转 = 前后点头/仰头
 *          → roll  = 绕 X 轴旋转 = 左右转头
 *          → yaw   = 绕 Z 轴旋转 = 侧倾（无磁力计不准，仅参考）
 */
#include "head_tracker.h"
#include <math.h>
#include <string.h>

static HeadAnalysis_t result;

/* ============ 可调阈值（实际佩戴后可根据效果微调） ============ */
static const float COMPENSATORY_ROLL_THRESHOLD = 20.0f;  /* roll 超此值视为代偿性转头 */
static const float TREMOR_STD_THRESHOLD        = 3.0f;   /* 头稳标准差超此值视为病理性头抖 */
static const float TILT_THRESHOLD              = 30.0f;  /* pitch 超此值视为异常前/后倾 */

/* ============ 内部状态 ============ */
static float prev_pitch = 0, prev_roll = 0, prev_yaw = 0;
static float pitch_buffer[100];       /* 最近 100 帧 pitch（用于计算头稳标准差） */
static float roll_buffer[100];        /* 最近 100 帧 roll */
static uint8_t buffer_idx   = 0;
static uint8_t buffer_full  = 0;
static uint8_t tremor_counter = 0;
static uint32_t last_update_ms = 0;

/**
 * @brief  初始化头部分析器
 *         清零所有内部缓存和输出结果
 */
void HeadTracker_Init(void)
{
    memset(&result, 0, sizeof(HeadAnalysis_t));
    memset(pitch_buffer, 0, sizeof(pitch_buffer));
    memset(roll_buffer,  0, sizeof(roll_buffer));
    buffer_idx   = 0;
    buffer_full  = 0;
    prev_pitch   = 0; prev_roll = 0; prev_yaw = 0;
    last_update_ms = 0;
}

/**
 * @brief  更新头部分析结果（每 10ms 调用一次）
 * @param  euler : BMI088 输出的欧拉角指针
 * @param  dt    : 时间步长（固定 0.01s）
 *
 *         分析流程（严格按此顺序执行）：
 *         1. 计算角度增量（用于衡量瞬时运动速率）
 *         2. 立即把 result.pitch/roll/yaw 更新为当前帧值（其后所有分析依赖此数据）
 *         3. 代偿性转头检测：|roll| > 20° → 患者用转代替换眼球运动
 *         4. 安全倾斜检测：|pitch| > 30° → 前后倾过度，触发暂停
 *         5. 循环缓冲区更新，计算 pitch/roll 联合标准差作为头稳指标
 *         6. 抖动检测：头稳连续 10 帧超 3° → 病理性头抖
 *         7. 姿势分类：根据 pitch/roll 绝对值判断抬/低/左偏/右偏/正中
 */
void HeadTracker_Update(bmi088_euler_data_t *euler, float dt)
{
    (void)dt;

    /* 1) 角度增量 — 表示头部在这一帧（10ms）内的旋转速度 */
    result.pitch_delta = euler->pitch - prev_pitch;
    result.roll_delta  = euler->roll  - prev_roll;
    result.yaw_delta   = euler->yaw   - prev_yaw;

    prev_pitch = euler->pitch;
    prev_roll  = euler->roll;
    prev_yaw   = euler->yaw;

    /* 2) 立即更新当前帧角度值（先赋值，后续姿势分类才能拿到正确值） */
    result.pitch = euler->pitch;
    result.roll  = euler->roll;
    result.yaw   = euler->yaw;

    /* 3) 代偿性转头检测
     *    板子竖放时 roll = 左右转头。偏盲患者眼睛无法扫视时，
     *    会用转头代替眼球运动，这是需要纠正的不良代偿行为。
     *    若 |roll| > 20°，标记为代偿行为，训练模式据此语音提醒 */
    float abs_roll = fabsf(result.roll);
    result.is_compensatory = (abs_roll > COMPENSATORY_ROLL_THRESHOLD) ? 1 : 0;

    /* 4) 安全倾斜检测
     *    仅检查 pitch（前后倾）。roll（左右转头）在训练中是正常行为，
     *    不应触发安全警报。若 |pitch| > 30°，说明患者严重前倾或后仰 */
    result.safety_alert = (fabsf(result.pitch) > TILT_THRESHOLD) ? 1 : 0;

    /* 5) 循环缓存 + 头稳指标
     *    将当前 pitch/roll 存入 100 帧环形缓冲区，计算最近 N 帧的
     *    联合标准差：头稳指标 = sqrt(var(pitch) + var(roll))
     *    指标越小 → 头部越稳；指标越大 → 晃动越厉害 */
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
            float dr = roll_buffer[i]  - r_mean;
            p_var += dp * dp;
            r_var += dr * dr;
        }
        p_var /= sample_count;
        r_var /= sample_count;

        result.head_stability = sqrtf(p_var + r_var);
    }

    /* 6) 抖动检测
     *    头稳指标连续 10 帧超过 3° 判定为病理性头抖。
     *    使用计数延迟防止瞬时波动误触发 */
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
     *    规则（基于已更新的 result.pitch/roll）：
     *    - pitch 绝对值 > 20° 且比 roll 大 → 抬头(pitch>0)/低头(pitch<0)
     *    - roll 绝对值  > 15° 且比 pitch 大 → 右偏(roll>0)/左偏(roll<0)
     *    - 否则 → 正中 */
    float abs_pitch = fabsf(result.pitch);
    abs_roll = fabsf(result.roll);

    if (abs_pitch > 20.0f && abs_pitch > abs_roll)
    {
        result.posture = (result.pitch > 0) ? POSTURE_UP : POSTURE_DOWN;
    }
    else if (abs_roll > 15.0f && abs_roll > abs_pitch)
    {
        result.posture = (result.roll > 0) ? POSTURE_RIGHT : POSTURE_LEFT;
    }
    else
    {
        result.posture = POSTURE_CENTER;
    }
}

/**
 * @brief  获取当前头部分析结果
 * @return HeadAnalysis_t 结构体指针
 */
HeadAnalysis_t* HeadTracker_GetResult(void)
{
    return &result;
}

/**
 * @brief  综合安全检查
 * @return 0=安全; 非 0=存在风险
 *         bit0 = 异常倾斜, bit1 = 头抖
 *         训练状态机根据返回值决定是否暂停训练
 */
uint8_t HeadTracker_CheckSafety(void)
{
    return result.safety_alert | result.is_tremor;
}
