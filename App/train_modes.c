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

#define APP_CALIB_CMD_GUARD_MS       1200U
#define APP_TRAIN_SAFETY_GRACE_MS     500U
#define APP_VOICE_LISTEN_WINDOW_MS   5000U
#define APP_MODE_DOUBLE_CLICK_MS      600U
#define APP_TRAIN_START_PROMPT_MS    4000U
#define APP_MODE_ENTER_PROMPT_MS     2500U

typedef enum {
    APP_EVENT_NONE = 0,
    APP_EVENT_ENTER_CALIB = 1,
    APP_EVENT_CALIB_CONFIRMED = 2,
    APP_EVENT_RESTART = 3,
    APP_EVENT_SKIP = 4,
    APP_EVENT_ENTER_PAUSE = 5,
    APP_EVENT_RESUME = 6,
    APP_EVENT_ENTER_FEEDBACK = 7,
    APP_EVENT_TRAIN_START = 8,
    APP_EVENT_ENTER_IDLE = 9,
    APP_EVENT_WAKE_HELLO = 10,
    APP_EVENT_CMD_IGNORED = 11,
    APP_EVENT_MODE_SELECT = 12,
    APP_EVENT_NEXT_CONFIRM = 13,
} AppEvent_t;

typedef enum {
    APP_FLOW_FULL = 0,
    APP_FLOW_CUSTOM = 1,
} AppFlowMode_t;

/* ===== 全局状态 ===== */
static SystemState_t   sys_state   = SYS_IDLE_VOICE;
static TrainMode_t     train_mode  = MODE_A_FIXATION;
static TrainingRecord_t record;
static AppFlowMode_t   flow_mode   = APP_FLOW_CUSTOM;

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
typedef enum {
    PURSUIT_MOVE = 0,
    PURSUIT_CHECKPOINT = 1,
} PursuitState_t;

#define PURSUIT_POINT_COUNT       8U
#define PURSUIT_MOVE_TIME_MS      2000U
#define PURSUIT_CHECK_TIME_MS     2000U
#define PURSUIT_COMP_LIMIT        20U

static PursuitState_t pursuit_state = PURSUIT_MOVE;
static uint8_t  pursuit_point_idx = 0;
static uint32_t pursuit_state_tick = 0;
static uint8_t  pursuit_start_x = 90;
static uint8_t  pursuit_start_y = 90;
static uint8_t  pursuit_target_x = 90;
static uint8_t  pursuit_target_y = 90;
static uint16_t pursuit_comp_count = 0;

/* ===== 手动姿态校准变量 ===== */
static uint8_t calib_prompted = 0;

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
static uint32_t pause_enter_tick    = 0;
static uint32_t paused_ms_last      = 0;
static uint8_t  pause_resume_x      = 90;
static uint8_t  pause_resume_y      = 90;
static uint8_t  pause_resume_laser  = 0;
static uint8_t  pause_resume_led    = 0;
static uint8_t  pause_by_voice      = 0;
static uint8_t  voice_listen_active = 0;
static uint32_t voice_listen_start_tick = 0;

/* ===== 运行诊断变量 ===== */
static uint8_t  last_voice_cmd      = 0;
static uint32_t last_voice_cmd_tick = 0;
static uint8_t  last_app_event      = APP_EVENT_NONE;
static uint32_t last_mode_cmd_tick  = 0;
static uint8_t  last_mode_cmd       = 0;
static uint32_t train_entry_tick    = 0;
static uint8_t  mode_select_prompted = 0;
static uint8_t  mode_select_clicks = 0;
static uint32_t mode_select_first_tick = 0;
static uint8_t  train_prompt_played = 0;
static uint32_t train_prompt_tick = 0;
static uint8_t  next_confirm_prompted = 0;
static uint8_t  next_confirm_advance = 1;
static AppFlowMode_t pending_flow_mode = APP_FLOW_CUSTOM;
static uint8_t  mode_enter_prompted = 0;
static uint32_t mode_enter_prompt_tick = 0;

/* ===== 应用层记录的舵机目标角度 ===== */
static uint8_t  app_servo_x = 90;
static uint8_t  app_servo_y = 90;
static uint8_t  app_laser_on = 0;
static uint8_t  app_led_focus_on = 0;

/* 舵机角度标定参数（由 Calibrate_ServoRange 测量后填入） */
/* X轴：第12步(80°)~第18步(110°)为训练范围 */
/* Y轴：舒适区约105~110°，取100~115° */
uint8_t CALIB_X_MIN = 80;    /* X轴训练范围左边界（患者右侧视野） */
uint8_t CALIB_X_MAX = 110;   /* X轴训练范围右边界（患者左侧视野） */
uint8_t CALIB_Y_MIN = 100;   /* Y轴训练范围底边 */
uint8_t CALIB_Y_MAX = 115;   /* Y轴训练范围顶边 */

/* 语音播报冷却计时 */
static uint32_t voice_cooldown = 0;

/* 完成模式计数（用于全部完成播报） */
static uint8_t  completed_modes  = 0;

/* 扫视连续正确计数 */
static uint8_t  saccade_streak   = 0;

/* 标定模式状态 */
static uint8_t  calib_phase    = 0;   /* 0=start 1/2=X轴两次标记 3/4=Y轴两次标记 5=完成 */
static uint8_t  calib_angle    = 70;  /* 当前扫描角度 */
static uint32_t calib_tick     = 0;   /* 步进计时 */
static uint8_t  calib_pressed  = 0;   /* 当前相位内按键次数 */
static uint8_t  calib_save1    = 0;   /* 第1次按键角度（不区分左右） */
static uint8_t  calib_save2    = 0;   /* 第2次按键角度 */

/* ===== 模式名语音 ID 映射 ===== */
static const uint8_t mode_voice_id[MODE_COUNT] = {
    VOICE_CMD_FIXATION,  /* A */
    VOICE_CMD_SACCADE,   /* B */
    VOICE_CMD_PURSUIT,   /* C */
    VOICE_CMD_FOCUS,     /* D */
    VOICE_CMD_NEGLECT    /* E */
};

static const uint8_t mode_start_tts_id[MODE_COUNT] = {
    VOICE_TTS_START_FIX,
    VOICE_TTS_START_SAC,
    VOICE_TTS_START_PUR,
    VOICE_TTS_START_FOCUS,
    VOICE_TTS_START_NEGLECT
};

/* 内部函数 */
static void App_State_ModeSelect(void);
static void App_State_ModeEnterPrompt(void);
static void App_State_Calibrate(void);
static void App_State_TrainPrompt(void);
static void App_State_Train(void);
static void App_State_Feedback(void);
static void App_State_Pause(void);
static void App_State_NextConfirm(void);

static void Train_Fixation(void);
static void Train_Saccade(void);
static void Train_Pursuit(void);
static void Train_Focus(void);
static void Train_Neglect(void);

static void App_Transition(SystemState_t next_state);
static void App_SafetyCheck(void);
static void App_SetEvent(AppEvent_t event);
static uint8_t App_ShouldIgnoreCalibCmd(uint8_t cmd);
static void App_SetServoAngle(uint8_t axis, uint8_t angle);
static void App_LaserOn(void);
static void App_LaserOff(void);
static void App_LEDFocusOn(void);
static void App_LEDFocusOff(void);
static void App_StopStimulus(void);
static void App_EnterPause(uint8_t play_posture_voice);
static void App_ResumeFromPause(void);
static void App_ApplyPauseTime(uint32_t paused_ms);
static void App_EnterVoiceListenPause(void);
static void App_CheckVoiceListenTimeout(void);
static void App_EnterModeSelect(void);
static void App_StartFullMode(void);
static void App_StartCustomMode(void);
static void App_RestartCurrentMode(void);
static void App_SkipToNextMode(void);
static void App_SelectModeAndCalibrate(TrainMode_t mode);
static uint8_t App_IsPauseCmd(uint8_t cmd);
static uint8_t App_IsResumeCmd(uint8_t cmd);
static uint8_t App_IsRestartCmd(uint8_t cmd);
static uint8_t App_IsSkipCmd(uint8_t cmd);
static uint8_t App_IsModeSwitchCmd(uint8_t cmd);
static TrainMode_t App_NextMode(TrainMode_t mode);
static uint8_t App_IsLastFullMode(void);
static void Pursuit_Reset(void);
static void Pursuit_SetTarget(uint8_t idx);
static void Pursuit_AdvancePoint(uint32_t now);
static float SmoothStep(float x);
static void State_CalibServo(void);
void Calibrate_ServoRange(void);

/**
 * @brief  初始化应用层
 */
void App_Init(void)
{
    sys_state  = SYS_MODE_SELECT;
    train_mode = MODE_A_FIXATION;
    flow_mode = APP_FLOW_CUSTOM;
    memset(&record, 0, sizeof(TrainingRecord_t));
    timebase = HAL_GetTick();
    completed_modes = 0;
    pause_enter_tick = 0;
    paused_ms_last = 0;
    pause_by_voice = 0;
    voice_listen_active = 0;
    voice_listen_start_tick = 0;
    last_voice_cmd = 0;
    last_voice_cmd_tick = 0;
    last_app_event = APP_EVENT_NONE;
    last_mode_cmd_tick = 0;
    last_mode_cmd = 0;
    train_entry_tick = 0;
    mode_select_prompted = 0;
    mode_select_clicks = 0;
    mode_select_first_tick = 0;
    train_prompt_played = 0;
    train_prompt_tick = 0;
    next_confirm_prompted = 0;
    next_confirm_advance = 1;
    pending_flow_mode = APP_FLOW_CUSTOM;
    mode_enter_prompted = 0;
    mode_enter_prompt_tick = 0;
    app_servo_x = 90;
    app_servo_y = 90;
    app_laser_on = 0;
    app_led_focus_on = 0;
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
    if (cmd != 0)
    {
        last_voice_cmd = cmd;
        last_voice_cmd_tick = HAL_GetTick();
    }

    if ((sys_state == SYS_CALIBRATE) && App_ShouldIgnoreCalibCmd(cmd))
        cmd = 0;

    if (cmd != 0 && App_IsModeSwitchCmd(cmd))
    {
        App_EnterModeSelect();
        return;
    }

    /* ===== 全局命令处理（唤醒词区 TYPE=0x01~0x0F）===== */
    if (VOICE_CMD_IS_WAKE(cmd))
    {
        /* "你好小盈" — 进入静音监听窗口，给后续命令留出说话时间 */
        if (cmd == VOICE_CMD_WAKE_HELLO)
        {
            if (sys_state == SYS_TRAIN || sys_state == SYS_PAUSE)
            {
                App_EnterVoiceListenPause();
                return;
            }
            App_SetEvent(APP_EVENT_WAKE_HELLO);
            return;
        }

        if (voice_listen_active && App_IsPauseCmd(cmd))
        {
            voice_listen_active = 0;
            pause_by_voice = 1;
            App_SetEvent(APP_EVENT_ENTER_PAUSE);
            return;
        }

        /* "继续训练" — 仅从可恢复暂停中恢复 */
        if (App_IsResumeCmd(cmd))
        {
            if (sys_state == SYS_PAUSE)
            {
                App_ResumeFromPause();
                return;
            }
            App_SetEvent(APP_EVENT_CMD_IGNORED);
            return;
        }

        /* 其他唤醒词命令按状态处理 */
        switch (sys_state)
        {
            case SYS_IDLE_VOICE:
            case SYS_FEEDBACK:
            case SYS_TRAIN:
                if (App_IsPauseCmd(cmd))
                {
                    if (sys_state == SYS_TRAIN)
                    {
                        App_EnterPause(1);
                        pause_by_voice = 1;
                    }
                    else
                    {
                        App_SetEvent(APP_EVENT_CMD_IGNORED);
                    }
                    return;
                }
                if (App_IsRestartCmd(cmd))
                {
                    App_RestartCurrentMode();
                    return;
                }
                if (App_IsSkipCmd(cmd))
                {
                    App_SkipToNextMode();
                    return;
                }
                break;

            case SYS_PAUSE:
                if (App_IsRestartCmd(cmd))
                {
                    App_RestartCurrentMode();
                    return;
                }
                if (App_IsSkipCmd(cmd))
                {
                    App_SkipToNextMode();
                    return;
                }
                break;

            default:
                App_SetEvent(APP_EVENT_CMD_IGNORED);
                break;
        }
    }

    /* ===== 命令词处理（TYPE=0x00，训练模式可跨状态切换）===== */
    if (cmd > 0 && !VOICE_CMD_IS_WAKE(cmd))
    {
        if ((sys_state == SYS_IDLE_VOICE || sys_state == SYS_TRAIN ||
             sys_state == SYS_PAUSE || sys_state == SYS_FEEDBACK) &&
            App_IsRestartCmd(cmd))
        {
            App_RestartCurrentMode();
            return;
        }
        if (sys_state == SYS_TRAIN && App_IsPauseCmd(cmd))
        {
            App_EnterPause(1);
            pause_by_voice = 1;
            voice_listen_active = 0;
            return;
        }
        if ((sys_state == SYS_PAUSE) && voice_listen_active && App_IsPauseCmd(cmd))
        {
            voice_listen_active = 0;
            pause_by_voice = 1;
            App_SetEvent(APP_EVENT_ENTER_PAUSE);
            return;
        }
        if (sys_state == SYS_PAUSE && App_IsResumeCmd(cmd))
        {
            App_ResumeFromPause();
            return;
        }
        if ((sys_state == SYS_IDLE_VOICE || sys_state == SYS_TRAIN ||
             sys_state == SYS_PAUSE || sys_state == SYS_FEEDBACK) &&
            App_IsSkipCmd(cmd))
        {
            App_SkipToNextMode();
            return;
        }

        if (sys_state == SYS_IDLE_VOICE)
        {
            if (cmd >= VOICE_CMD_CALIB_MODE && cmd <= VOICE_CMD_CALIB_MODE)
            {
                calib_phase   = 0;
                calib_pressed = 0;
                sys_state = SYS_CALIB_SERVO;
                return;
            }
        }

        if ((sys_state == SYS_IDLE_VOICE || sys_state == SYS_TRAIN ||
             sys_state == SYS_PAUSE || sys_state == SYS_FEEDBACK) &&
            (cmd >= VOICE_CMD_FIXATION && cmd <= VOICE_CMD_NEGLECT))
        {
            App_SelectModeAndCalibrate((TrainMode_t)(cmd - VOICE_CMD_FIXATION));
            return;
        }

        if (cmd != 0)
        {
            App_SetEvent(APP_EVENT_CMD_IGNORED);
        }
    }

    App_CheckVoiceListenTimeout();

    /* ===== 状态机分发 ===== */
    switch (sys_state)
    {
        case SYS_IDLE_VOICE: break;
        case SYS_MODE_SELECT: App_State_ModeSelect(); break;
        case SYS_MODE_ENTER_PROMPT: App_State_ModeEnterPrompt(); break;
        case SYS_CALIBRATE:  App_State_Calibrate(); break;
        case SYS_TRAIN_PROMPT: App_State_TrainPrompt(); break;
        case SYS_TRAIN:
            if (train_mode != MODE_A_FIXATION)
                App_SafetyCheck();
            if (sys_state == SYS_TRAIN)
                App_State_Train();
            break;
        case SYS_FEEDBACK:   App_State_Feedback();  break;
        case SYS_PAUSE:      App_State_Pause();     break;
        case SYS_NEXT_CONFIRM: App_State_NextConfirm(); break;
        case SYS_CALIB_SERVO: State_CalibServo();   break;
    }
}

/* ============ 状态查询 ============ */
SystemState_t App_GetState(void) { return sys_state; }
TrainMode_t   App_GetMode(void)  { return train_mode; }
TrainingRecord_t App_GetRecord(void) { return record; }
uint8_t App_GetLastVoiceCmd(void) { return last_voice_cmd; }
uint32_t App_GetLastVoiceCmdTick(void) { return last_voice_cmd_tick; }
uint32_t App_GetPauseEnterTick(void) { return pause_enter_tick; }
uint32_t App_GetLastPausedMs(void) { return paused_ms_last; }
uint8_t App_GetLastEvent(void) { return last_app_event; }

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

static void App_State_ModeSelect(void)
{
    uint32_t now = HAL_GetTick();
    KeyEvent_t key_event;

    if (!mode_select_prompted)
    {
        App_StopStimulus();
        (void)Key_GetEvent(KEY_PATIENT);
        Voice_Play(0xFF, VOICE_TTS_MODE_SELECT);
        voice_cooldown = now;
        mode_select_prompted = 1;
        mode_select_clicks = 0;
        mode_select_first_tick = 0;
        return;
    }

    key_event = Key_GetEvent(KEY_PATIENT);
    if (key_event == KEY_EVENT_SHORT)
    {
        if (mode_select_clicks == 0)
        {
            mode_select_clicks = 1;
            mode_select_first_tick = now;
        }
        else
        {
            App_StartCustomMode();
            return;
        }
    }

    if (mode_select_clicks == 1 && (now - mode_select_first_tick) >= APP_MODE_DOUBLE_CLICK_MS)
    {
        App_StartFullMode();
    }
}

static void App_State_ModeEnterPrompt(void)
{
    uint32_t now = HAL_GetTick();

    if (!mode_enter_prompted)
    {
        App_StopStimulus();
        Voice_Play(0xFF, (pending_flow_mode == APP_FLOW_FULL) ?
                   VOICE_TTS_ENTER_FULL_MODE : VOICE_TTS_ENTER_CUSTOM_MODE);
        voice_cooldown = now;
        mode_enter_prompt_tick = now;
        mode_enter_prompted = 1;
        return;
    }

    if ((now - mode_enter_prompt_tick) < APP_MODE_ENTER_PROMPT_MS)
        return;

    mode_enter_prompted = 0;
    flow_mode = pending_flow_mode;
    completed_modes = 0;

    if (flow_mode == APP_FLOW_FULL)
    {
        train_mode = MODE_A_FIXATION;
        App_Transition(SYS_CALIBRATE);
    }
    else
    {
        App_Transition(SYS_IDLE_VOICE);
    }
}

/**
 * @brief  校准状态
 */
static void App_State_Calibrate(void)
{
    if (!calib_prompted)
    {
        App_StopStimulus();
        (void)Key_GetEvent(KEY_PATIENT);
        Voice_Play(0xFF, VOICE_TTS_START_CALIB);
        voice_cooldown = HAL_GetTick();
        calib_prompted = 1;
        return;
    }

    if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT)
    {
        BMI088_euler_init();
        HeadTracker_Init();
        App_SetEvent(APP_EVENT_CALIB_CONFIRMED);
        Voice_Play(0xFF, VOICE_TTS_CALIB_DONE);
        voice_cooldown = HAL_GetTick();
        calib_prompted = 0;
        App_Transition(SYS_TRAIN_PROMPT);
    }
}

static void App_State_TrainPrompt(void)
{
    uint32_t now = HAL_GetTick();

    if (!train_prompt_played)
    {
        App_StopStimulus();
        Voice_Play(0xFF, mode_start_tts_id[(uint8_t)train_mode]);
        voice_cooldown = now;
        train_prompt_tick = now;
        train_prompt_played = 1;
        return;
    }

    if ((now - train_prompt_tick) >= APP_TRAIN_START_PROMPT_MS)
    {
        train_prompt_played = 0;
        App_Transition(SYS_TRAIN);
    }
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

    if (record.completed)
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
        {
            if (total == 0) total = 1;
            uint8_t rate = correct * 100 / total;
            if (rate < 50)
                Voice_Play(0xFF, VOICE_TTS_RESULT_TRY);
            else if (pursuit_comp_count > PURSUIT_COMP_LIMIT)
                Voice_Play(0xFF, VOICE_TTS_PUR_HEAD);
            else
                Voice_Play(0xFF, VOICE_TTS_PUR_GOOD);
            HAL_Delay(8000);
            break;
        }

        default:
            Voice_Play(0xFF, VOICE_TTS_TRAIN_DONE);
            HAL_Delay(8000);
            break;
    }

    if (flow_mode == APP_FLOW_FULL && App_IsLastFullMode())
    {
        Voice_Play(0xFF, VOICE_TTS_ALL_DONE);
        HAL_Delay(4000);
        completed_modes = 0;
        App_EnterModeSelect();
        return;
    }

    if (flow_mode == APP_FLOW_FULL)
    {
        next_confirm_advance = 1;
        App_Transition(SYS_NEXT_CONFIRM);
    }
    else
    {
        App_Transition(SYS_IDLE_VOICE);
    }
}

static void App_State_NextConfirm(void)
{
    if (!next_confirm_prompted)
    {
        App_StopStimulus();
        (void)Key_GetEvent(KEY_PATIENT);
        Voice_Play(0xFF, VOICE_TTS_NEXT_CONFIRM);
        voice_cooldown = HAL_GetTick();
        next_confirm_prompted = 1;
        return;
    }

    if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT)
    {
        next_confirm_prompted = 0;
        if (next_confirm_advance)
            train_mode = App_NextMode(train_mode);
        App_Transition(SYS_CALIBRATE);
    }
}

/**
 * @brief  暂停状态：姿态异常时自动触发，姿态正常 3 秒后恢复
 */
static void App_State_Pause(void)
{
    uint32_t now = HAL_GetTick();

    if (pause_by_voice)
    {
        return;
    }

    if (!pause_voice_played)
    {
        Voice_Play(0xFF, VOICE_TTS_KEEP_STILL);
        voice_cooldown = HAL_GetTick();
        Buzzer_Alert(2, 150, 100);
        pause_voice_played = 1;
        return;
    }

    /* 检查姿态是否恢复 */
    HeadAnalysis_t *head = HeadTracker_GetResult();
    uint8_t alert = 0;
    /* Vertical mount semantics: roll=nod, pitch=head turn, yaw=lateral tilt. */
    if (fabsf(head->roll)  > 20.0f) alert = 1;
    if (fabsf(head->pitch) > 20.0f) alert = 1;
    if (fabsf(head->yaw)   > 30.0f) alert = 1;

    if (!alert)
    {
        if (pause_stable_tick == 0)
            pause_stable_tick = now;

        if (now - pause_stable_tick >= 3000)
        {
            App_ResumeFromPause();
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
    if ((HAL_GetTick() - train_entry_tick) < APP_TRAIN_SAFETY_GRACE_MS)
        return;

    HeadAnalysis_t *head = HeadTracker_GetResult();
    uint8_t alert = 0;
    /* Vertical mount semantics: roll=nod, pitch=head turn, yaw=lateral tilt. */
    if (fabsf(head->roll)  > 20.0f) alert = 1;
    if (fabsf(head->pitch) > 20.0f) alert = 1;
    if (fabsf(head->yaw)   > 30.0f) alert = 1;

    if (alert)
    {
        App_EnterPause(0);
    }
}

static void App_SetServoAngle(uint8_t axis, uint8_t angle)
{
    if (angle > 180) angle = 180;
    Servo_SetAngle(axis, angle);
    if (axis == SERVO_AXIS_X)
        app_servo_x = angle;
    else if (axis == SERVO_AXIS_Y)
        app_servo_y = angle;
}

static void App_SetEvent(AppEvent_t event)
{
    last_app_event = (uint8_t)event;
}

static uint8_t App_ShouldIgnoreCalibCmd(uint8_t cmd)
{
    if (cmd == 0)
        return 0;

    if (!App_IsRestartCmd(cmd) && !App_IsSkipCmd(cmd))
        return 0;

    if (cmd != last_mode_cmd)
        return 0;

    return (uint8_t)((HAL_GetTick() - last_mode_cmd_tick) < APP_CALIB_CMD_GUARD_MS);
}

static void App_LaserOn(void)
{
    Laser_On();
    app_laser_on = 1;
}

static void App_LaserOff(void)
{
    Laser_Off();
    app_laser_on = 0;
}

static void App_LEDFocusOn(void)
{
    LED_On(LED_FOCUS);
    app_led_focus_on = 1;
}

static void App_LEDFocusOff(void)
{
    LED_Off(LED_FOCUS);
    app_led_focus_on = 0;
}

static void App_StopStimulus(void)
{
    App_LaserOff();
    App_LEDFocusOff();
    App_SetServoAngle(SERVO_AXIS_X, 90);
    App_SetServoAngle(SERVO_AXIS_Y, 90);
}

static void App_EnterPause(uint8_t play_posture_voice)
{
    if (sys_state == SYS_PAUSE)
        return;

    App_SetEvent(APP_EVENT_ENTER_PAUSE);
    pause_resume_x = app_servo_x;
    pause_resume_y = app_servo_y;
    pause_resume_laser = app_laser_on;
    pause_resume_led = app_led_focus_on;
    pause_enter_tick = HAL_GetTick();
    pause_stable_tick = 0;
    pause_voice_played = 0;
    pause_by_voice = 0;

    if (play_posture_voice)
    {
        Voice_Play(0xFF, VOICE_TTS_POSTURE);
        voice_cooldown = HAL_GetTick();
        Buzzer_Alert(2, 150, 100);
        pause_voice_played = 1;
    }

    App_LaserOff();
    App_LEDFocusOff();
    App_SetServoAngle(SERVO_AXIS_X, 90);
    App_SetServoAngle(SERVO_AXIS_Y, 90);
    sys_state = SYS_PAUSE;
}

static void App_ResumeFromPause(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t paused_ms = 0;

    if (pause_enter_tick != 0)
        paused_ms = now - pause_enter_tick;

    paused_ms_last = paused_ms;
    App_ApplyPauseTime(paused_ms);
    App_SetEvent(APP_EVENT_RESUME);

    Voice_Play(0xFF, VOICE_TTS_CONTINUE);
    voice_cooldown = HAL_GetTick();
    App_SetServoAngle(SERVO_AXIS_X, pause_resume_x);
    App_SetServoAngle(SERVO_AXIS_Y, pause_resume_y);
    if (pause_resume_laser)
        App_LaserOn();
    else
        App_LaserOff();
    if (pause_resume_led)
        App_LEDFocusOn();
    else
        App_LEDFocusOff();

    pause_voice_played = 0;
    pause_stable_tick = 0;
    pause_enter_tick = 0;
    pause_by_voice = 0;
    voice_listen_active = 0;
    sys_state = SYS_TRAIN;
}

static void App_ApplyPauseTime(uint32_t paused_ms)
{
    if (paused_ms == 0)
        return;

    timebase += paused_ms;
    record.start_tick += paused_ms;

    if (trial_start_tick != 0)
        trial_start_tick += paused_ms;
    if (saccade_light_on_tick != 0)
        saccade_light_on_tick += paused_ms;
    if (pursuit_state_tick != 0)
        pursuit_state_tick += paused_ms;
    if (focus_phase_tick != 0)
        focus_phase_tick += paused_ms;
    if (neglect_trial_tick != 0)
        neglect_trial_tick += paused_ms;
}

static void App_EnterVoiceListenPause(void)
{
    if (sys_state == SYS_TRAIN)
    {
        App_EnterPause(0);
    }

    if (sys_state == SYS_PAUSE)
    {
        pause_by_voice = 1;
        voice_listen_active = 1;
        voice_listen_start_tick = HAL_GetTick();
        App_SetEvent(APP_EVENT_WAKE_HELLO);
    }
}

static void App_CheckVoiceListenTimeout(void)
{
    if (!voice_listen_active)
        return;

    if (sys_state != SYS_PAUSE || !pause_by_voice)
    {
        voice_listen_active = 0;
        return;
    }

    if ((HAL_GetTick() - voice_listen_start_tick) >= APP_VOICE_LISTEN_WINDOW_MS)
    {
        App_ResumeFromPause();
    }
}

static void App_EnterModeSelect(void)
{
    App_StopStimulus();
    record.completed = 0;
    pause_stable_tick = 0;
    pause_voice_played = 0;
    pause_enter_tick = 0;
    pause_by_voice = 0;
    voice_listen_active = 0;
    mode_select_prompted = 0;
    mode_select_clicks = 0;
    mode_select_first_tick = 0;
    next_confirm_prompted = 0;
    App_Transition(SYS_MODE_SELECT);
}

static void App_StartFullMode(void)
{
    pending_flow_mode = APP_FLOW_FULL;
    mode_select_prompted = 0;
    mode_select_clicks = 0;
    mode_enter_prompted = 0;
    App_Transition(SYS_MODE_ENTER_PROMPT);
}

static void App_StartCustomMode(void)
{
    pending_flow_mode = APP_FLOW_CUSTOM;
    mode_select_prompted = 0;
    mode_select_clicks = 0;
    mode_enter_prompted = 0;
    App_Transition(SYS_MODE_ENTER_PROMPT);
}

static void App_RestartCurrentMode(void)
{
    last_mode_cmd = last_voice_cmd;
    last_mode_cmd_tick = HAL_GetTick();
    App_StopStimulus();
    record.completed = 0;
    record.end_tick = HAL_GetTick();
    pause_stable_tick = 0;
    pause_voice_played = 0;
    pause_enter_tick = 0;
    pause_by_voice = 0;
    voice_listen_active = 0;
    App_Transition(SYS_CALIBRATE);
    App_SetEvent(APP_EVENT_RESTART);
}

static TrainMode_t App_NextMode(TrainMode_t mode)
{
    uint8_t next = (uint8_t)mode + 1U;
    if (next >= MODE_COUNT)
        next = MODE_A_FIXATION;
    return (TrainMode_t)next;
}

static void App_SkipToNextMode(void)
{
    last_mode_cmd = last_voice_cmd;
    last_mode_cmd_tick = HAL_GetTick();
    App_StopStimulus();
    record.completed = 0;
    record.end_tick = HAL_GetTick();
    train_mode = App_NextMode(train_mode);
    pause_stable_tick = 0;
    pause_voice_played = 0;
    pause_enter_tick = 0;
    pause_by_voice = 0;
    voice_listen_active = 0;

    if (flow_mode == APP_FLOW_FULL)
    {
        next_confirm_advance = 0;
        App_Transition(SYS_NEXT_CONFIRM);
    }
    else
    {
        Voice_Play(0xFF, VOICE_TTS_NEXT_PREP);
        voice_cooldown = HAL_GetTick();
        App_Transition(SYS_CALIBRATE);
    }
    App_SetEvent(APP_EVENT_SKIP);
}

static void App_SelectModeAndCalibrate(TrainMode_t mode)
{
    if ((uint8_t)mode >= (uint8_t)MODE_COUNT)
        return;

    App_StopStimulus();
    train_mode = mode;
    record.completed = 0;
    record.end_tick = HAL_GetTick();
    pause_stable_tick = 0;
    pause_voice_played = 0;
    pause_enter_tick = 0;
    pause_by_voice = 0;
    voice_listen_active = 0;
    HAL_Delay(600);
    App_Transition(SYS_CALIBRATE);
}

static uint8_t App_IsPauseCmd(uint8_t cmd)
{
    return (cmd == VOICE_CMD_WAKE_PAUSE);
}

static uint8_t App_IsResumeCmd(uint8_t cmd)
{
    return (cmd == VOICE_CMD_WAKE_RESUME);
}

static uint8_t App_IsRestartCmd(uint8_t cmd)
{
    return (cmd == VOICE_CMD_WAKE_RESTART || cmd == VOICE_CMD_RESTART);
}

static uint8_t App_IsSkipCmd(uint8_t cmd)
{
    return (cmd == VOICE_CMD_WAKE_SKIP || cmd == VOICE_CMD_SKIP);
}

static uint8_t App_IsModeSwitchCmd(uint8_t cmd)
{
    return (cmd == VOICE_CMD_MODE_SWITCH);
}

static uint8_t App_IsLastFullMode(void)
{
    return (train_mode == MODE_E_NEGLECT);
}

static float SmoothStep(float x)
{
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x * x * (3.0f - 2.0f * x);
}

static void Pursuit_SetTarget(uint8_t idx)
{
    uint8_t x_mid = (uint8_t)(((uint16_t)CALIB_X_MIN + (uint16_t)CALIB_X_MAX) / 2U);
    uint8_t y_mid = (uint8_t)(((uint16_t)CALIB_Y_MIN + (uint16_t)CALIB_Y_MAX) / 2U);

    switch (idx)
    {
        case 0: pursuit_target_x = CALIB_X_MIN; pursuit_target_y = y_mid; break;
        case 1: pursuit_target_x = CALIB_X_MAX; pursuit_target_y = y_mid; break;
        case 2: pursuit_target_x = x_mid;       pursuit_target_y = CALIB_Y_MAX; break;
        case 3: pursuit_target_x = x_mid;       pursuit_target_y = CALIB_Y_MIN; break;
        case 4: pursuit_target_x = CALIB_X_MIN; pursuit_target_y = CALIB_Y_MAX; break;
        case 5: pursuit_target_x = CALIB_X_MAX; pursuit_target_y = CALIB_Y_MIN; break;
        case 6: pursuit_target_x = CALIB_X_MAX; pursuit_target_y = CALIB_Y_MAX; break;
        case 7: pursuit_target_x = CALIB_X_MIN; pursuit_target_y = CALIB_Y_MIN; break;
        default: pursuit_target_x = x_mid;      pursuit_target_y = y_mid; break;
    }
}

static void Pursuit_Reset(void)
{
    pursuit_state = PURSUIT_MOVE;
    pursuit_point_idx = 0;
    pursuit_state_tick = HAL_GetTick();
    pursuit_start_x = 90;
    pursuit_start_y = 90;
    pursuit_comp_count = 0;
    Pursuit_SetTarget(0);
}

static void Pursuit_AdvancePoint(uint32_t now)
{
    pursuit_point_idx++;
    if (pursuit_point_idx >= PURSUIT_POINT_COUNT)
    {
        App_StopStimulus();
        record.end_tick = now;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
        return;
    }

    pursuit_start_x = pursuit_target_x;
    pursuit_start_y = pursuit_target_y;
    Pursuit_SetTarget(pursuit_point_idx);
    pursuit_state = PURSUIT_MOVE;
    pursuit_state_tick = now;
}

/**
 * @brief  状态切换
 */
static void App_Transition(SystemState_t next_state)
{
    sys_state = next_state;

    if (next_state == SYS_CALIBRATE)
    {
        App_SetEvent(APP_EVENT_ENTER_CALIB);
        calib_prompted = 0;
        train_prompt_played = 0;
        pause_stable_tick = 0;
        pause_voice_played = 0;
        pause_by_voice = 0;
        voice_listen_active = 0;
    }

    if (next_state == SYS_TRAIN_PROMPT)
    {
        train_prompt_played = 0;
        train_prompt_tick = 0;
    }

    if (next_state == SYS_TRAIN)
    {
        App_SetEvent(APP_EVENT_TRAIN_START);
        memset(&record, 0, sizeof(TrainingRecord_t));
        record.mode = train_mode;
        record.start_tick = HAL_GetTick();
        timebase = HAL_GetTick();
        train_entry_tick = record.start_tick;
        Pursuit_Reset();
        focus_phase = 0;
        saccade_idx = 0;
        saccade_count = 0;
        neglect_trial_count = 0;
        neglect_trial_tick = 0;
        pause_stable_tick = 0;
        pause_voice_played = 0;
        voice_cooldown = 0;

        /* 模式播报由模块自动响应语音命令词完成，MCU 只需等待即可 */
        saccade_light_on_tick = 0;
    }

    if (next_state == SYS_FEEDBACK)
    {
        App_SetEvent(APP_EVENT_ENTER_FEEDBACK);
    }

    if (next_state == SYS_IDLE_VOICE)
    {
        App_SetEvent(APP_EVENT_ENTER_IDLE);
    }

    if (next_state == SYS_MODE_SELECT)
    {
        App_SetEvent(APP_EVENT_MODE_SELECT);
        mode_select_prompted = 0;
        mode_select_clicks = 0;
        mode_select_first_tick = 0;
        mode_enter_prompted = 0;
    }

    if (next_state == SYS_MODE_ENTER_PROMPT)
    {
        mode_enter_prompted = 0;
        mode_enter_prompt_tick = 0;
    }

    if (next_state == SYS_NEXT_CONFIRM)
    {
        App_SetEvent(APP_EVENT_NEXT_CONFIRM);
        next_confirm_prompted = 0;
    }
}

/**
 * @brief  A — 注视稳定性训练
 *         激光固定点，患者注视 15 秒，检测头稳指标
 */
static void Train_Fixation(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - timebase;

    App_LaserOn();
    App_LEDFocusOn();

    HeadAnalysis_t *head = HeadTracker_GetResult();
    if (head->head_stability > 5.0f)
    {
        if (HAL_GetTick() - voice_cooldown > 3000)
        {
            Voice_Play(0xFF, VOICE_TTS_KEEP_STILL);
            voice_cooldown = HAL_GetTick();
            HAL_Delay(500);
        }
    }

    if (elapsed > 15000)
    {
        App_LaserOff();
        App_LEDFocusOff();
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
        uint8_t x_angle = CALIB_X_MIN, y_angle = CALIB_Y_MIN;
        switch (current_target)
        {
            case 0: x_angle = CALIB_X_MIN; y_angle = CALIB_Y_MIN; break;
            case 1: x_angle = CALIB_X_MAX; y_angle = CALIB_Y_MIN; break;
            case 2: x_angle = CALIB_X_MIN; y_angle = CALIB_Y_MAX; break;
            case 3: x_angle = CALIB_X_MAX; y_angle = CALIB_Y_MAX; break;
        }

        App_SetServoAngle(SERVO_AXIS_X, x_angle);
        App_SetServoAngle(SERVO_AXIS_Y, y_angle);
        HAL_Delay(100);
        App_LaserOn();
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
        App_LaserOff();
        saccade_streak++;
        if (saccade_streak >= 3)
        {
            Voice_Play(0xFF, VOICE_TTS_STREAK);
            saccade_streak = 0;
        }
        else
        {
            Voice_Play(0xFF, VOICE_TTS_CORRECT);
        }
        voice_cooldown = HAL_GetTick();
        HAL_Delay(1000);
        saccade_light_on_tick = 0;
        saccade_idx++;
        HAL_Delay(500);
    }

    /* 3 秒超时 */
    if (now - saccade_light_on_tick > 3000 && trial_result == 0)
    {
        saccade_streak = 0;
        App_LaserOff();
        Voice_Play(0xFF, VOICE_TTS_TIMEOUT);
        voice_cooldown = HAL_GetTick();
        HAL_Delay(1000);
        saccade_light_on_tick = 0;
        saccade_idx++;
        HAL_Delay(500);
    }
}

/**
 * @brief  C — 平稳追踪训练
 *         激光平滑移动到关键点，到点后等待 PA2 确认
 */
static void Train_Pursuit(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - pursuit_state_tick;

    App_LaserOn();

    HeadAnalysis_t *head = HeadTracker_GetResult();
    if (head->is_compensatory)
        pursuit_comp_count++;

    if (pursuit_state == PURSUIT_MOVE)
    {
        float s = (float)elapsed / (float)PURSUIT_MOVE_TIME_MS;
        if (s > 1.0f) s = 1.0f;
        s = SmoothStep(s);

        uint8_t x_angle = (uint8_t)((float)pursuit_start_x +
                           ((float)pursuit_target_x - (float)pursuit_start_x) * s);
        uint8_t y_angle = (uint8_t)((float)pursuit_start_y +
                           ((float)pursuit_target_y - (float)pursuit_start_y) * s);
        App_SetServoAngle(SERVO_AXIS_X, x_angle);
        App_SetServoAngle(SERVO_AXIS_Y, y_angle);

        if (elapsed >= PURSUIT_MOVE_TIME_MS)
        {
            App_SetServoAngle(SERVO_AXIS_X, pursuit_target_x);
            App_SetServoAngle(SERVO_AXIS_Y, pursuit_target_y);
            pursuit_state = PURSUIT_CHECKPOINT;
            pursuit_state_tick = now;
            trial_start_tick = now;
            trial_result = 0;
            record.total_trials++;
            Voice_Play(0xFF, VOICE_TTS_FIND_LIGHT);
            voice_cooldown = HAL_GetTick();
        }
        return;
    }

    if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && trial_result == 0)
    {
        trial_result = 1;
        record.correct_trials++;
        record.avg_reaction_ms += (float)(now - trial_start_tick);
        Voice_Play(0xFF, VOICE_TTS_CORRECT);
        voice_cooldown = HAL_GetTick();
        Pursuit_AdvancePoint(now);
        return;
    }

    if (elapsed >= PURSUIT_CHECK_TIME_MS && trial_result == 0)
    {
        Voice_Play(0xFF, VOICE_TTS_TIMEOUT);
        voice_cooldown = HAL_GetTick();
        Pursuit_AdvancePoint(now);
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
        if ((focus_phase & 1) == 0) { App_LEDFocusOn(); App_LaserOff(); }
        else                        { App_LEDFocusOff(); App_LaserOn(); }

        if (elapsed > 5000) { focus_phase++; focus_phase_tick = now; }
    }
    else
    {
        App_LEDFocusOff();
        App_LaserOff();
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

        App_SetServoAngle(SERVO_AXIS_X, neglect_side ? CALIB_X_MAX : CALIB_X_MIN);
        HAL_Delay(300);
        App_LaserOn();
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
        App_LaserOff();
        App_SetServoAngle(SERVO_AXIS_X, 90);
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
        App_LaserOff();
        App_SetServoAngle(SERVO_AXIS_X, 90);
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

/**
 * @brief  标定模式 — 按键标记视野边界
 *         语音命令「标定模式」触发，先标X轴（从70°向右扫描→按PA2标记边界）
 *         再标Y轴（向上扫描→按PA2标记边界），最后回到IDLE_VOICE
 *
 *         标定结果直接修改 CALIB_X/Y_MIN/MAX，训练模式自动使用新值。
 *         不重启则不丢，重启后还原为代码默认值。
 *
 *  相序:  0=开始语音 → 1=X右边界 → 2=X左边界 → 3=Y底边 → 4=Y顶边 → 5=完成
 */
static void State_CalibServo(void)
{
    uint8_t v;

    switch (calib_phase)
    {
        case 0:
            Buzzer_Alert(2, 150, 100);
            Laser_On();
            Servo_SetAngle(SERVO_AXIS_Y, 90);
            Servo_SetAngle(SERVO_AXIS_X, 70);
            HAL_Delay(2000);
            calib_angle   = 70;
            calib_pressed = 0;
            calib_save1   = 0;
            calib_save2   = 0;
            calib_tick    = HAL_GetTick();
            calib_phase   = 1;
            break;

        /* ===== X轴标定（70°→150°→70°）===== */
        case 1:
            if (HAL_GetTick() - calib_tick < 100) return;
            calib_tick = HAL_GetTick();
            if (calib_angle < 150) calib_angle++;
            Servo_SetAngle(SERVO_AXIS_X, calib_angle);

            if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && calib_pressed == 0)
            {
                calib_save1   = calib_angle;
                calib_pressed = 1;
                Buzzer_Alert(1, 100, 0);
            }
            if (calib_angle >= 150)
            {
                if (calib_pressed == 0) calib_save1 = 110;
                calib_phase   = 2;
                calib_angle   = 150;
                calib_pressed = 0;
            }
            break;

        case 2:
            if (HAL_GetTick() - calib_tick < 100) return;
            calib_tick = HAL_GetTick();
            if (calib_angle > 70) calib_angle--;
            Servo_SetAngle(SERVO_AXIS_X, calib_angle);

            if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && calib_pressed == 0)
            {
                calib_save2   = calib_angle;
                calib_pressed = 1;
                Buzzer_Alert(1, 100, 0);
            }
            if (calib_angle <= 70)
            {
                if (calib_pressed == 0) calib_save2 = 110;
                v = calib_save1; calib_save1 = (v < calib_save2) ? v : calib_save2;
                v = calib_save2; calib_save2 = (v > calib_save1) ? v : calib_save1;
                CALIB_X_MIN = calib_save1;
                CALIB_X_MAX = calib_save2;

                Servo_SetAngle(SERVO_AXIS_X, 90);
                calib_phase   = 3;
                calib_angle   = 80;
                calib_pressed = 0;
                calib_save1   = 0;
                calib_save2   = 0;
                Buzzer_Alert(2, 150, 100);
                HAL_Delay(1500);
            }
            break;

        /* ===== Y轴标定（80°→140°→80°）===== */
        case 3:
            if (HAL_GetTick() - calib_tick < 100) return;
            calib_tick = HAL_GetTick();
            if (calib_angle < 140) calib_angle++;
            Servo_SetAngle(SERVO_AXIS_Y, calib_angle);

            if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && calib_pressed == 0)
            {
                calib_save1   = calib_angle;
                calib_pressed = 1;
                Buzzer_Alert(1, 100, 0);
            }
            if (calib_angle >= 140)
            {
                if (calib_pressed == 0) calib_save1 = 100;
                calib_phase   = 4;
                calib_angle   = 140;
                calib_pressed = 0;
            }
            break;

        case 4:
            if (HAL_GetTick() - calib_tick < 100) return;
            calib_tick = HAL_GetTick();
            if (calib_angle > 80) calib_angle--;
            Servo_SetAngle(SERVO_AXIS_Y, calib_angle);

            if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && calib_pressed == 0)
            {
                calib_save2   = calib_angle;
                calib_pressed = 1;
                Buzzer_Alert(1, 100, 0);
            }
            if (calib_angle <= 80)
            {
                if (calib_pressed == 0) calib_save2 = 115;
                v = calib_save1; calib_save1 = (v < calib_save2) ? v : calib_save2;
                v = calib_save2; calib_save2 = (v > calib_save1) ? v : calib_save1;
                CALIB_Y_MIN = calib_save1;
                CALIB_Y_MAX = calib_save2;

                calib_phase = 5;
            }
            break;

        /* ===== 完成 ===== */
        case 5:
            Laser_Off();
            Servo_SetAngle(SERVO_AXIS_X, 90);
            Servo_SetAngle(SERVO_AXIS_Y, 90);
            Voice_Play(0xFF, VOICE_TTS_INIT_OK);
            HAL_Delay(2500);
            calib_phase   = 0;
            calib_pressed = 0;
            sys_state = SYS_IDLE_VOICE;
            break;
    }
}

/**
 * @brief  舵机角度范围标定（旧版自动扫描）
 *         已废弃，请使用语音命令「标定模式」触发的 State_CalibServo
 */
void Calibrate_ServoRange(void)
{
    State_CalibServo();
}
