#include "head_tracker.h"
#include <math.h>
#include <string.h>

static HeadAnalysis_t result;

static const float COMPENSATORY_YAW_THRESHOLD  = 15.0f;
static const float TREMOR_STD_THRESHOLD        = 3.0f;
static const float TILT_THRESHOLD              = 30.0f;
static const float STABILITY_WINDOW            = 1.0f;

static float prev_pitch = 0, prev_roll = 0, prev_yaw = 0;
static float pitch_buffer[100], roll_buffer[100];
static uint8_t buffer_idx = 0, buffer_full = 0;
static float elapsed = 0;
static uint8_t tremor_counter = 0;
static uint32_t last_update_ms = 0;
static float last_pitch_valid = 0, last_roll_valid = 0;

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

void HeadTracker_Update(bmi088_euler_data_t *euler, float dt)
{
    uint32_t now = HAL_GetTick();
    float dt_ms = (float)(now - last_update_ms);
    last_update_ms = now;

    result.pitch_delta = euler->pitch - prev_pitch;
    result.roll_delta  = euler->roll  - prev_roll;
    result.yaw_delta   = euler->yaw   - prev_yaw;

    prev_pitch = euler->pitch;
    prev_roll  = euler->roll;
    prev_yaw   = euler->yaw;

    float abs_yaw = fabsf(euler->yaw);
    if (abs_yaw > COMPENSATORY_YAW_THRESHOLD)
    {
        result.is_compensatory = 1;
    }
    else
    {
        result.is_compensatory = 0;
    }

    if (fabsf(euler->pitch) > TILT_THRESHOLD || fabsf(euler->roll) > TILT_THRESHOLD)
    {
        result.safety_alert = 1;
    }
    else
    {
        result.safety_alert = 0;
    }

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

HeadAnalysis_t* HeadTracker_GetResult(void)
{
    return &result;
}

uint8_t HeadTracker_CheckSafety(void)
{
    return result.safety_alert | result.is_tremor;
}
