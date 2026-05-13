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
static uint32_t pause_stable_tick   = 0;
static uint8_t  pause_voice_played  = 0;

/* 语音播报冷却计时（防止每 10ms 循环重复触发同一句） */
static uint32_t voice_cooldown = 0;

/* 完成模式计数（用于全部完成播报） */
static uint8_t  completed_modes  = 0;

/* 扫视连续正确计数 */
static uint8_t  saccade_streak   = 0;

/* ===== 模式名语音 ID 映射 ===== */
static const uint8_t mode_voice_id[MODE_COUNT] = {
    VOICE_CMD_FIXATION,  /* A */
    VOICE_CMD_SACCADE,   /* B */
    VOICE_CMD_PURSUIT,   /* C */
    VOICE_CMD_FOCUS,     /* D */
    VOICE_CMD_NEGLECT    /* E */
};

/* 内部函数 */
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
    completed_modes = 0;
}

/**
 * @brief  应用主运行函数（每 10ms 主循环调用）
 *         全局处理语音命令 + 状态分发
 */
void App_Run(bmi088_euler_data_t *euler, float temp)
{
    (void)temp; (void)euler;

    /* 读取语音命令 */
    uint8_t cmd = Voice_GetCommand();

    /* ===== 全局命令处理（唤醒词区 TYPE=0x01~0x0F）===== */
    if (VOICE_CMD_IS_WAKE(cmd))
    {
        /* "你好小盈" — 统一中断信号，停止一切回到 IDLE_VOICE */
        if (cmd == VOICE_CMD_WAKE_HELLO)
        {
            if (sys_state == SYS_TRAIN || sys_state == SYS_PAUSE)
            {
                Laser_Off();
                Servo_SetAngle(SERVO_AXIS_X, 90);
                Servo_SetAngle(SERVO_AXIS_Y, 90);
                Voice_Play(0xFF, VOICE_TTS_POSTURE);
            }
            sys_state = SYS_IDLE_VOICE;
            pause_voice_played = 0;
            pause_stable_tick = 0;
            return;
        }

        /* "继续训练" — 从 IDLE_VOICE 恢复之前的训练 */
        if (cmd == VOICE_CMD_WAKE_RESUME)
        {
            if (sys_state == SYS_PAUSE)
            {
                Voice_Play(0xFF, VOICE_TTS_CONTINUE);
                pause_voice_played  = 0;
                pause_stable_tick   = 0;
                sys_state = SYS_TRAIN;
                return;
            }
            if (sys_state == SYS_IDLE_VOICE)
            {
                sys_state = SYS_TRAIN;
                return;
            }
        }

        /* 其他唤醒词命令按状态处理 */
        switch (sys_state)
        {
            case SYS_TRAIN:
                if (cmd == VOICE_CMD_WAKE_PAUSE)
                {
                    Voice_Play(0xFF, VOICE_TTS_POSTURE);
                    Buzzer_Alert(2, 150, 100);
                    Laser_Off();
                    Servo_SetAngle(SERVO_AXIS_X, 90);
                    Servo_SetAngle(SERVO_AXIS_Y, 90);
                    sys_state = SYS_PAUSE;
                    return;
                }
                if (cmd == VOICE_CMD_WAKE_RESTART)
                {
                    App_Transition(SYS_CALIBRATE);
                    return;
                }
                if (cmd == VOICE_CMD_WAKE_SKIP)
                {
                    record.end_tick = HAL_GetTick();
                    record.completed = 1;
                    App_Transition(SYS_FEEDBACK);
                    return;
                }
                break;

            case SYS_PAUSE:
                if (cmd == VOICE_CMD_WAKE_SKIP)
                {
                    record.end_tick = HAL_GetTick();
                    record.completed = 1;
                    App_Transition(SYS_FEEDBACK);
                    return;
                }
                break;

            default:
                break;
        }
    }

    /* ===== 命令词处理（TYPE=0x00，仅在 IDLE_VOICE 中有效）===== */
    if (cmd > 0 && !VOICE_CMD_IS_WAKE(cmd))
    {
        if (sys_state == SYS_IDLE_VOICE &&
            cmd >= VOICE_CMD_FIXATION && cmd <= VOICE_CMD_NEGLECT)
        {
            train_mode = (TrainMode_t)(cmd - VOICE_CMD_FIXATION);
            HAL_Delay(600);
            App_Transition(SYS_CALIBRATE);
            return;
        }
    }

    /* ===== 状态机分发 ===== */
    switch (sys_state)
    {
        case SYS_IDLE_VOICE: break;  /* 无命令时静默等待 */
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
 * @brief  IDLE_VOICE — 语音命令在 App_Run 中全局处理
 *         此状态不做任何事，静默等待即可
 */

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
    voice_cooldown = HAL_GetTick();
    HAL_Delay(3500);
    BMI088_euler_init();
    Voice_Play(0xFF, VOICE_TTS_CALIB_DONE);
    voice_cooldown = HAL_GetTick();
    HAL_Delay(2500);
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
    uint16_t total   = record.total_trials;
    uint16_t correct = record.correct_trials;
    (void)total; (void)correct;

    completed_modes++;

    switch (train_mode)
    {
        case MODE_A_FIXATION:
            if (record.avg_head_stability < 3.0f)
                Voice_Play(0xFF, VOICE_TTS_FIX_GOOD);
            else if (record.avg_head_stability < 5.0f)
                Voice_Play(0xFF, VOICE_TTS_FIX_FAIR);
            else
                Voice_Play(0xFF, VOICE_TTS_FIX_POOR);
            HAL_Delay(8000);
            break;

        case MODE_B_SACCADE:
        {
            if (total == 0) total = 1;
            uint8_t rate = correct * 100 / total;
            if (rate >= 80)
            {
                Voice_Play(0xFF, VOICE_TTS_RESULT_GREAT);
                HAL_Delay(2000);
                if (record.avg_reaction_ms / correct < 1500.0f)
                    Voice_Play(0xFF, VOICE_TTS_REACT_FAST);
                else
                    Voice_Play(0xFF, VOICE_TTS_REACT_SLOW);
            }
            else if (rate >= 50)
                Voice_Play(0xFF, VOICE_TTS_RESULT_GOOD);
            else
                Voice_Play(0xFF, VOICE_TTS_RESULT_TRY);
            HAL_Delay(8000);
            break;
        }

        case MODE_C_PURSUIT:
            if (pursuit_phase > 6)
                Voice_Play(0xFF, VOICE_TTS_PUR_HEAD);
            else
                Voice_Play(0xFF, VOICE_TTS_PUR_GOOD);
            HAL_Delay(8000);
            break;

        default:
            Voice_Play(0xFF, VOICE_TTS_TRAIN_DONE);
            HAL_Delay(8000);
            break;
    }

    if (completed_modes >= 5)
    {
        Voice_Play(0xFF, VOICE_TTS_ALL_DONE);
        HAL_Delay(4000);
        completed_modes = 0;
    }

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
        voice_cooldown = HAL_GetTick();
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
            Voice_Play(0xFF, VOICE_TTS_CONTINUE);
            voice_cooldown = HAL_GetTick();
            HAL_Delay(2000);
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
        voice_cooldown = 0;

        /* 模式播报由模块自动响应语音命令词完成，MCU 只需等待即可 */
        HAL_Delay(1200);
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
    {
        if (HAL_GetTick() - voice_cooldown > 3000)
        {
            Voice_Play(0xFF, VOICE_TTS_EYE_ONLY);
            voice_cooldown = HAL_GetTick();
            HAL_Delay(500);
        }
    }

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
        Voice_Play(0xFF, neglect_side ? VOICE_TTS_RIGHT_SIDE : VOICE_TTS_LEFT_SIDE);
        voice_cooldown = HAL_GetTick();
        HAL_Delay(2000);
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
        Voice_Play(0xFF, VOICE_TTS_FOUND_SIDE);
        voice_cooldown = HAL_GetTick();
        neglect_trial_tick = 0;
        HAL_Delay(2500);
        return;
    }

    if (now - neglect_trial_tick > 5000 && !neglect_responded)
    {
        Voice_Play(0xFF, VOICE_TTS_NEGLECT_HINT);
        voice_cooldown = HAL_GetTick();
        Laser_Off();
        Servo_SetAngle(SERVO_AXIS_X, 90);
        neglect_trial_tick = 0;
        HAL_Delay(1000);
        return;
    }

    if (neglect_trial_count >= 6)
    {
        record.end_tick = now;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
    }
}
