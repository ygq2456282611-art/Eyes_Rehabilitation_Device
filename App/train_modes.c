/**
 * @file    train_modes.c
 * @brief   语音驱动的训练模式状态机
 *
 *          流程：
 *          SYS_IDLE_VOICE → 轮询 Voice_GetCommand()
 *          → 语音命令匹配 → 播报模式名 → CALIBRATE → TRAIN → FEEDBACK → IDLE
 *
 *          安全暂停：训练中检测到姿态异常 → SYS_PAUSE（激光灭、舵机回中）
 *          → 姿态连续正常 3 秒 → 自动恢复训练
 *
 *          患者反馈：PA2 按键（短按），用于扫视/忽略训练中确认目标
 */
#include "train_modes.h"
#include "servo.h"
#include "voice.h"
#include "laser.h"
#include "key.h"
#include "led.h"
#include "head_tracker.h"
#include "buzzer.h"
#include <string.h>

/* ===== 全局状态 ===== */
static SystemState_t   sys_state   = SYS_IDLE_VOICE;
static TrainMode_t     train_mode  = MODE_A_FIXATION;
static TrainingRecord_t record;

/* ===== 训练内部时间基准 ===== */
static uint32_t timebase        = 0;
static uint32_t trial_start_tick = 0;
static uint8_t  trial_result    = 0;

/* ===== 扫视训练变量 ===== */
static uint8_t  saccade_seq[20];
static uint8_t  saccade_idx         = 0;
static uint8_t  saccade_count       = 0;
static uint8_t  current_target      = 0;
static uint32_t saccade_light_on_tick = 0;

/* ===== 追踪训练变量 ===== */
static uint8_t pursuit_phase = 0;

/* ===== 聚焦训练变量 ===== */
static uint8_t  focus_phase      = 0;
static uint32_t focus_phase_tick = 0;

/* ===== 空间忽略训练变量 ===== */
static uint8_t  neglect_side        = 0;
static uint32_t neglect_trial_tick  = 0;
static uint8_t  neglect_responded   = 0;
static uint8_t  neglect_trial_count = 0;

/* ===== 安全暂停/恢复变量 ===== */
static uint32_t pause_enter_tick    = 0;
static uint32_t pause_stable_tick   = 0;
static uint8_t  pause_voice_played  = 0;

/* ===== 模式名语音 ID 映射 ===== */
static const uint8_t mode_voice_id[MODE_COUNT] = {
    VOICE_CMD_FIXATION,  /* A */
    VOICE_CMD_SACCADE,   /* B */
    VOICE_CMD_PURSUIT,   /* C */
    VOICE_CMD_FOCUS,     /* D */
    VOICE_CMD_NEGLECT    /* E */
};

/* 内部函数 */
static void App_State_IdleVoice(void);
static void App_State_Calibrate(void);
static void App_State_Train(void);
static void App_State_Feedback(void);
static void App_State_Pause(void);

static void Train_Fixation(void);
static void Train_Saccade(void);
static void Train_Pursuit(void);
static void Train_Focus(void);
static void Train_Neglect(void);

static void App_Transition(SystemState_t next_state);
static void App_SafetyCheck(void);

/**
 * @brief  初始化应用层
 */
void App_Init(void)
{
    sys_state  = SYS_IDLE_VOICE;
    train_mode = MODE_A_FIXATION;
    memset(&record, 0, sizeof(TrainingRecord_t));
    timebase = HAL_GetTick();
}

/**
 * @brief  应用主运行函数（每 10ms 主循环调用）
 */
void App_Run(bmi088_euler_data_t *euler, float temp)
{
    (void)temp; (void)euler;

    switch (sys_state)
    {
        case SYS_IDLE_VOICE: App_State_IdleVoice(); break;
        case SYS_CALIBRATE:  App_State_Calibrate(); break;
        case SYS_TRAIN:      App_SafetyCheck(); App_State_Train(); break;
        case SYS_FEEDBACK:   App_State_Feedback();  break;
        case SYS_PAUSE:      App_State_Pause();     break;
    }
}

/* ============ 状态查询 ============ */
SystemState_t App_GetState(void) { return sys_state; }
TrainMode_t   App_GetMode(void)  { return train_mode; }
TrainingRecord_t App_GetRecord(void) { return record; }

/**
 * @brief  IDLE_VOICE — 语音命令等待
 *         轮询 Voice_GetCommand()，收到有效命令词后进入校准
 */
static void App_State_IdleVoice(void)
{
    uint8_t cmd = Voice_GetCommand();
    if (cmd == 0) return;

    /* 映射命令词到训练模式 */
    if (cmd >= VOICE_CMD_FIXATION && cmd <= VOICE_CMD_NEGLECT)
    {
        train_mode = (TrainMode_t)(cmd - VOICE_CMD_FIXATION);
    }
    else
    {
        return;  /* 无效命令词，忽略 */
    }

    /* 播报确认：命令词本身已经触发模块播报，MCU 加一个小停顿 */
    HAL_Delay(600);
    App_Transition(SYS_CALIBRATE);
}

/**
 * @brief  校准状态
 */
static void App_State_Calibrate(void)
{
    Voice_Play(0x00, VOICE_CMD_CALIB_STILL);
    HAL_Delay(2000);
    BMI088_euler_init();
    Voice_Play(0xFF, VOICE_TTS_CALIB_DONE);
    HAL_Delay(1000);
    App_Transition(SYS_TRAIN);
}

/**
 * @brief  训练状态：分发到对应模式
 */
static void App_State_Train(void)
{
    switch (train_mode)
    {
        case MODE_A_FIXATION: Train_Fixation(); break;
        case MODE_B_SACCADE:  Train_Saccade();  break;
        case MODE_C_PURSUIT:  Train_Pursuit();  break;
        case MODE_D_FOCUS:    Train_Focus();    break;
        case MODE_E_NEGLECT:  Train_Neglect();  break;
        default: break;
    }
}

/**
 * @brief  反馈状态：播报结果 → 回到 IDLE
 */
static void App_State_Feedback(void)
{
    Voice_Play(0xFF, VOICE_TTS_TRAIN_DONE);
    HAL_Delay(2500);
    App_Transition(SYS_IDLE_VOICE);
}

/**
 * @brief  暂停状态：姿态异常时自动触发，姿态正常 3 秒后恢复
 */
static void App_State_Pause(void)
{
    uint32_t now = HAL_GetTick();

    if (!pause_voice_played)
    {
        Voice_Play(0xFF, VOICE_TTS_KEEP_STILL);
        Buzzer_Alert(2, 150, 100);
        Laser_Off();
        Servo_SetAngle(SERVO_AXIS_X, 90);
        Servo_SetAngle(SERVO_AXIS_Y, 90);
        pause_voice_played = 1;
        return;
    }

    /* 检查姿态是否恢复 */
    HeadAnalysis_t *head = HeadTracker_GetResult();
    uint8_t alert = 0;
    if (fabsf(head->roll)  > 20.0f) alert = 1;
    if (fabsf(head->pitch) > 20.0f) alert = 1;
    if (fabsf(head->yaw)   > 30.0f) alert = 1;

    if (!alert)
    {
        if (pause_stable_tick == 0)
            pause_stable_tick = now;

        if (now - pause_stable_tick >= 3000)
        {
            Voice_Play(0xFF, VOICE_TTS_CORRECT);
            HAL_Delay(800);
            Laser_On();
            pause_voice_played  = 0;
            pause_stable_tick   = 0;
            sys_state = SYS_TRAIN;
        }
    }
    else
    {
        pause_stable_tick = 0;
    }
}

/**
 * @brief  安全检查：训练中检测姿态异常 → 暂停
 */
static void App_SafetyCheck(void)
{
    HeadAnalysis_t *head = HeadTracker_GetResult();
    uint8_t alert = 0;
    if (fabsf(head->roll)  > 20.0f) alert = 1;
    if (fabsf(head->pitch) > 20.0f) alert = 1;
    if (fabsf(head->yaw)   > 30.0f) alert = 1;

    if (alert)
    {
        sys_state = SYS_PAUSE;
        pause_enter_tick  = HAL_GetTick();
        pause_stable_tick = 0;
        pause_voice_played = 0;
    }
}

/**
 * @brief  状态切换
 */
static void App_Transition(SystemState_t next_state)
{
    sys_state = next_state;

    if (next_state == SYS_TRAIN)
    {
        memset(&record, 0, sizeof(TrainingRecord_t));
        record.mode = train_mode;
        record.start_tick = HAL_GetTick();
        timebase = HAL_GetTick();
        pursuit_phase = 0;
        focus_phase = 0;
        saccade_idx = 0;
        saccade_count = 0;
        neglect_trial_count = 0;
        neglect_trial_tick = 0;
        pause_stable_tick = 0;
        pause_voice_played = 0;

        /* 播报当前模式 */
        Voice_Play(0x00, mode_voice_id[train_mode]);
    }

    if (next_state == SYS_IDLE_VOICE)
    {
        Laser_Off();
        LED_Off(LED_FOCUS);
        LED_Off(LED_STATUS);
    }
}

/* ==================== 五种训练模式 ==================== */

/**
 * @brief  A — 注视稳定性训练
 *         激光固定点，患者注视 15 秒，检测头稳指标
 */
static void Train_Fixation(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - timebase;

    Laser_On();
    LED_On(LED_FOCUS);

    HeadAnalysis_t *head = HeadTracker_GetResult();
    if (head->head_stability > 5.0f)
    {
        Voice_Play(0xFF, VOICE_TTS_KEEP_STILL);
    }

    if (elapsed > 15000)
    {
        Laser_Off();
        LED_Off(LED_FOCUS);
        record.end_tick = now;
        record.avg_head_stability = head->head_stability;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
    }
}

/**
 * @brief  B — 扫视训练
 *         激光 4 位置随机跳变 × 8 次
 *         患者看到激光按 PA2 确认 → 记录反应时间
 */
static void Train_Saccade(void)
{
    uint32_t now = HAL_GetTick();

    if (saccade_count == 0)
    {
        saccade_count = 8;
        for (uint8_t i = 0; i < saccade_count; i++)
            saccade_seq[i] = i % 4;
        for (uint8_t i = saccade_count - 1; i > 0; i--)
        {
            uint8_t j = (uint8_t)(HAL_GetTick() % (i + 1));
            uint8_t tmp = saccade_seq[i];
            saccade_seq[i] = saccade_seq[j];
            saccade_seq[j] = tmp;
        }
        saccade_idx = 0;
    }

    if (saccade_light_on_tick == 0)
    {
        if (saccade_idx >= saccade_count)
        {
            record.end_tick = now;
            record.completed = 1;
            App_Transition(SYS_FEEDBACK);
            return;
        }

        current_target = saccade_seq[saccade_idx];
        uint8_t x_angle = 45, y_angle = 60;
        switch (current_target)
        {
            case 0: x_angle = 45;  y_angle = 60;  break;
            case 1: x_angle = 135; y_angle = 60;  break;
            case 2: x_angle = 45;  y_angle = 120; break;
            case 3: x_angle = 135; y_angle = 120; break;
        }

        Servo_SetAngle(SERVO_AXIS_X, x_angle);
        Servo_SetAngle(SERVO_AXIS_Y, y_angle);
        HAL_Delay(100);
        Laser_On();
        saccade_light_on_tick = now;
        trial_start_tick = now;
        trial_result = 0;
        record.total_trials++;
    }

    /* 患者 PA2 按键确认 */
    if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && trial_result == 0)
    {
        trial_result = 1;
        record.correct_trials++;
        record.avg_reaction_ms += (float)(now - trial_start_tick);
        Laser_Off();
        Voice_Play(0xFF, VOICE_TTS_CORRECT);
        saccade_light_on_tick = 0;
        saccade_idx++;
        HAL_Delay(500);
    }

    /* 3 秒超时 */
    if (now - saccade_light_on_tick > 3000 && trial_result == 0)
    {
        Laser_Off();
        Voice_Play(0xFF, VOICE_TTS_TIMEOUT);
        saccade_light_on_tick = 0;
        saccade_idx++;
        HAL_Delay(500);
    }
}

/**
 * @brief  C — 平稳追踪训练
 *         激光连续移动（水平→斜线→圆）× 30 秒
 */
static void Train_Pursuit(void)
{
    uint32_t now = HAL_GetTick();
    float t = (float)(now - timebase) / 5000.0f;

    Laser_On();

    uint8_t x_angle, y_angle;
    switch (pursuit_phase % 3)
    {
        case 0: x_angle = (uint8_t)(90 + 60 * t); y_angle = 90;
            if (t >= 1.0f) { pursuit_phase++; timebase = now; } break;
        case 1: x_angle = (uint8_t)(150*(1-t) + 30*t); y_angle = (uint8_t)(60*(1-t) + 120*t);
            if (t >= 1.0f) { pursuit_phase++; timebase = now; } break;
        case 2:
        { float rad = t * 6.2832f;
          x_angle = (uint8_t)(90 + 60*cosf(rad)); y_angle = (uint8_t)(90 + 30*sinf(rad));
          if (t >= 1.0f) { pursuit_phase++; timebase = now; } break;
        }
    }

    if (x_angle > 180) x_angle = 180;
    if (y_angle > 180) y_angle = 180;

    Servo_SetAngle(SERVO_AXIS_X, x_angle);
    Servo_SetAngle(SERVO_AXIS_Y, y_angle);

    HeadAnalysis_t *head = HeadTracker_GetResult();
    if (head->is_compensatory)
        Voice_Play(0xFF, VOICE_TTS_EYE_ONLY);

    if (now - record.start_tick > 30000)
    {
        Laser_Off();
        Servo_SetAngle(SERVO_AXIS_X, 90);
        Servo_SetAngle(SERVO_AXIS_Y, 90);
        record.end_tick = now;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
    }
}

/**
 * @brief  D — 视觉聚焦训练
 *         近距 LED / 远距激光交替 5s × 5 轮
 */
static void Train_Focus(void)
{
    uint32_t now = HAL_GetTick();
    if (focus_phase_tick == 0) focus_phase_tick = now;
    uint32_t elapsed = now - focus_phase_tick;

    if (focus_phase < 10)
    {
        if ((focus_phase & 1) == 0) { LED_On(LED_FOCUS); Laser_Off(); }
        else                        { LED_Off(LED_FOCUS); Laser_On(); }

        if (elapsed > 5000) { focus_phase++; focus_phase_tick = now; }
    }
    else
    {
        LED_Off(LED_FOCUS);
        Laser_Off();
        record.end_tick = now;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
    }
}

/**
 * @brief  E — 空间忽略训练
 *         左右视野交替点亮激光，患者按 PA2 确认
 */
static void Train_Neglect(void)
{
    uint32_t now = HAL_GetTick();

    if (neglect_trial_tick == 0)
    {
        neglect_side = neglect_trial_count & 1;

        Servo_SetAngle(SERVO_AXIS_X, neglect_side ? 150 : 30);
        HAL_Delay(300);
        Laser_On();
        Voice_Play(0xFF, VOICE_TTS_FIND_LIGHT);
        neglect_trial_tick = now;
        neglect_responded = 0;
        record.total_trials++;
        neglect_trial_count++;
        return;
    }

    uint32_t reaction = now - neglect_trial_tick;

    if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && !neglect_responded)
    {
        neglect_responded = 1;
        record.correct_trials++;
        record.avg_reaction_ms += (float)reaction;
        Laser_Off();
        Servo_SetAngle(SERVO_AXIS_X, 90);
        Voice_Play(0xFF, VOICE_TTS_CORRECT);
        neglect_trial_tick = 0;
        HAL_Delay(1500);
        return;
    }

    if (now - neglect_trial_tick > 5000 && !neglect_responded)
    {
        Voice_Play(0xFF, VOICE_TTS_NEGLECT_HINT);
        Laser_Off();
        Servo_SetAngle(SERVO_AXIS_X, 90);
        neglect_trial_tick = 0;
        HAL_Delay(1000);
        return;
    }

    if (neglect_trial_count >= 10)
    {
        record.end_tick = now;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
    }
}
