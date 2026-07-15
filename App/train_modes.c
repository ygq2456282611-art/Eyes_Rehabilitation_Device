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
#include <math.h>
#include <string.h>

#define APP_CALIB_CMD_GUARD_MS       1200U
#define APP_TRAIN_SAFETY_GRACE_MS     500U
#define APP_VOICE_LISTEN_WINDOW_MS   5000U
#define APP_MODE_DOUBLE_CLICK_MS      600U
#define APP_TRAIN_START_PROMPT_MS    4000U
#define APP_MODE_ENTER_PROMPT_MS     2500U
#define APP_MODE_CMD_ACK_WAIT_MS     1800U

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
    APP_EVENT_MODE_CMD_ACK_WAIT = 14,
    APP_EVENT_ENTER_SERVO_CALIB = 15,
    APP_EVENT_SERVO_CALIB_DONE = 16,
    APP_EVENT_CONTROL_STOP = 17,
} AppEvent_t;

/* ===== 全局状态 ===== */
static SystemState_t   sys_state   = SYS_IDLE_VOICE;
static TrainMode_t     train_mode  = MODE_A_FIXATION;
static TrainingRecord_t record;
static AppFlowMode_t   flow_mode   = APP_FLOW_CUSTOM;
static TrainingConfig_t training_config;
static TrainingPlan_t active_plan;
static TrainingSessionReport_t session_report;
static ModeReport_t mode_reports[MODE_COUNT];
static TrialResult_t trial_results[MODE_COUNT][APP_MAX_TRIALS_PER_MODE];
static uint8_t trial_counts[MODE_COUNT];
static uint8_t active_plan_index;
static uint8_t session_active;
static uint32_t next_session_id = 1U;

typedef struct {
    uint32_t sample_count;
    float offset_sum_deg;
    float variation_sum_deg;
    uint32_t compensatory_samples;
    uint32_t last_sample_tick;
} ModeMetricAccumulator_t;

static ModeMetricAccumulator_t mode_metrics;
static float live_head_offset_deg;

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
static uint8_t  saccade_phase = 0; /* 0=准备目标 1=等待到位 2=等待响应 3=间隔 */
static uint32_t saccade_phase_tick = 0;

/* ===== 追踪训练变量 ===== */
typedef enum {
    PURSUIT_MOVE = 0,
    PURSUIT_CHECKPOINT = 1,
} PursuitState_t;

#define PURSUIT_POINT_COUNT       8U

static PursuitState_t pursuit_state = PURSUIT_MOVE;
static uint8_t  pursuit_point_idx = 0;
static uint32_t pursuit_state_tick = 0;
static uint8_t  pursuit_start_x = 90;
static uint8_t  pursuit_start_y = 90;
static uint8_t  pursuit_target_x = 90;
static uint8_t  pursuit_target_y = 90;
static uint8_t pursuit_total_points = PURSUIT_POINT_COUNT;
static uint32_t pursuit_move_time_ms = 2000U;

/* ===== 手动姿态校准变量 ===== */
static uint8_t calib_prompted = 0;

/* ===== 聚焦训练变量 ===== */
static uint8_t  focus_phase      = 0;
static uint32_t focus_phase_tick = 0;
static uint8_t  focus_target_active = 0; /* 0=准备 1=等待PA2 2=轮次间隔 */

/* ===== 空间忽略训练变量 ===== */
static uint8_t  neglect_side        = 0;
static uint32_t neglect_trial_tick  = 0;
static uint8_t  neglect_responded   = 0;
static uint8_t  neglect_trial_count = 0;
static uint8_t  neglect_phase       = 0; /* 0=准备 1=等待到位 2=等待响应 3=间隔 */
static uint32_t neglect_phase_tick  = 0;

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
static uint8_t  pause_by_control    = 0;
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
static uint32_t mode_cmd_ack_tick = 0;
static uint8_t  servo_range_calibrated = 0;
static uint8_t  servo_calib_prompted = 0;
static uint8_t  servo_calib_after_mode_enter = 0;
static uint8_t  feedback_phase = 0;
static uint32_t feedback_tick = 0;

/* ===== 应用层记录的舵机目标角度 ===== */
static uint8_t  app_servo_x = 90;
static uint8_t  app_servo_y = 90;
static uint8_t  app_laser_on = 0;
static uint8_t  app_led_focus_on = 0;

/* 舵机角度标定参数（由 Calibrate_ServoRange 测量后填入） */
/* X轴：第12步(80°)~第18步(110°)为训练范围 */
/* Y轴默认以实物水平位置 90° 为中心，首次标定前仅使用窄范围。 */
uint8_t CALIB_X_MIN = 80;    /* X轴训练范围左边界（患者右侧视野） */
uint8_t CALIB_X_MAX = 110;   /* X轴训练范围右边界（患者左侧视野） */
uint8_t CALIB_Y_MIN = 75;    /* Y轴当前训练底边，标定后更新 */
uint8_t CALIB_Y_MAX = 135;   /* Y轴当前训练顶边，标定后更新 */

/* 语音播报冷却计时 */
static uint32_t voice_cooldown = 0;

/* 完成模式计数（用于全部完成播报） */
static uint8_t  completed_modes  = 0;

/* 扫视连续正确计数 */
static uint8_t  saccade_streak   = 0;

/* 标定模式状态 */
static uint8_t  calib_phase    = 0;   /* 0=start 1/2=X轴 3/4=Y轴 5=校验 6=完成提示 */
static uint32_t calib_wait_until = 0;
static uint8_t  calib_angle    = 30;  /* 当前扫描角度 */
static uint32_t calib_tick     = 0;   /* 步进计时 */
static uint8_t  calib_pressed  = 0;   /* 当前相位内按键次数 */
static uint8_t  calib_save1    = 0;   /* 第1次按键角度（不区分左右） */
static uint8_t  calib_save2    = 0;   /* 第2次按键角度 */
static uint8_t  calib_point_count = 0; /* 本轮有效记录点数，必须满4点 */
static uint8_t  calib_x_min_pending = 0;
static uint8_t  calib_x_max_pending = 0;
static uint8_t  calib_y_min_pending = 0;
static uint8_t  calib_y_max_pending = 0;

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
static void App_State_ModeCmdAckWait(void);
static void App_State_ServoCalibReady(void);
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
static uint8_t App_GetServoCenterX(void);
static uint8_t App_GetServoCenterY(void);
static void App_SetServoCenter(void);
static void App_SetServoPark(void);
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
static void App_StopFromControl(void);
static void App_EnterServoCalibReady(uint8_t after_mode_enter);
static void App_StartFullMode(void);
static void App_StartCustomMode(void);
static void App_RestartCurrentMode(void);
static void App_SkipToNextMode(void);
static void App_SelectModeAndCalibrate(TrainMode_t mode);
static void App_StartPlanMode(uint8_t index);
static uint8_t App_ValidatePlan(const TrainingPlan_t *plan);
static void App_ResetSessionReport(const TrainingPlan_t *plan);
static void App_ResetModeReport(TrainMode_t mode);
static void App_UpdateModeMetrics(void);
static void App_RecordTrial(uint8_t target, uint8_t side, uint8_t correct,
                            uint8_t timed_out, uint32_t reaction_ms);
static void App_CompleteCurrentMode(void);
static void App_FinalizeModeReport(void);
static uint8_t App_CalculateModeScore(const ModeReport_t *report);
static uint8_t App_CalculateHeadScore(const ModeReport_t *report);
static uint8_t App_CalculateReactionScore(const ModeReport_t *report);
static uint8_t App_IsSessionLastMode(void);
static uint8_t App_NextSessionMode(void);
static uint8_t App_ConfigIsValid(const TrainingConfig_t *config,
                                 AppControlResult_t *result);
static uint8_t App_IsPauseCmd(uint8_t cmd);
static uint8_t App_IsResumeCmd(uint8_t cmd);
static uint8_t App_IsRestartCmd(uint8_t cmd);
static uint8_t App_IsSkipCmd(uint8_t cmd);
static uint8_t App_IsModeSwitchCmd(uint8_t cmd);
static TrainMode_t App_NextMode(TrainMode_t mode);
static void Pursuit_Reset(void);
static void Pursuit_SetTarget(uint8_t idx);
static void Pursuit_AdvancePoint(uint32_t now);
static uint8_t Pursuit_GetBasePointCount(void);
static uint32_t Pursuit_CalculateMoveTime(void);
static float SmoothStep(float x);
static void State_CalibServo(void);
void Calibrate_ServoRange(void);

/**
 * @brief  初始化应用层
 */
void App_Init(void)
{
    App_GetDefaultTrainingConfig(&training_config);
    sys_state  = SYS_MODE_SELECT;
    train_mode = MODE_A_FIXATION;
    flow_mode = APP_FLOW_CUSTOM;
    memset(&record, 0, sizeof(TrainingRecord_t));
    timebase = HAL_GetTick();
    completed_modes = 0;
    pause_enter_tick = 0;
    paused_ms_last = 0;
    pause_by_voice = 0;
    pause_by_control = 0;
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
    mode_cmd_ack_tick = 0;
    servo_range_calibrated = 0;
    servo_calib_prompted = 0;
    servo_calib_after_mode_enter = 0;
    calib_point_count = 0;
    calib_x_min_pending = 0;
    calib_x_max_pending = 0;
    calib_y_min_pending = 0;
    calib_y_max_pending = 0;
    app_servo_x = 90;
    app_servo_y = 90;
    app_laser_on = 0;
    app_led_focus_on = 0;
    memset(&active_plan, 0, sizeof(active_plan));
    memset(&session_report, 0, sizeof(session_report));
    memset(mode_reports, 0, sizeof(mode_reports));
    memset(trial_results, 0, sizeof(trial_results));
    memset(trial_counts, 0, sizeof(trial_counts));
    memset(&mode_metrics, 0, sizeof(mode_metrics));
    active_plan_index = 0U;
    session_active = 0U;
    live_head_offset_deg = 0.0f;
    /* 保持 Servo_Init 设置的 90 度安全停放位，不在上电时应用训练中心。 */
}

/**
 * @brief  应用主运行函数（每 10ms 主循环调用）
 *         全局处理语音命令 + 状态分发
 */
void App_Run(bmi088_euler_data_t *euler, float temp)
{
    (void)temp;
    if (euler != 0)
        live_head_offset_deg = sqrtf(euler->pitch * euler->pitch + euler->roll * euler->roll);

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
        if (cmd == VOICE_CMD_CALIB_MODE)
        {
            App_EnterServoCalibReady(0);
            return;
        }

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
        case SYS_MODE_CMD_ACK_WAIT: App_State_ModeCmdAckWait(); break;
        case SYS_SERVO_CALIB_READY: App_State_ServoCalibReady(); break;
        case SYS_CALIBRATE:  App_State_Calibrate(); break;
        case SYS_TRAIN_PROMPT: App_State_TrainPrompt(); break;
        case SYS_TRAIN:
            if (train_mode != MODE_A_FIXATION)
                App_SafetyCheck();
            if (sys_state == SYS_TRAIN)
            {
                App_UpdateModeMetrics();
                App_State_Train();
            }
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

void App_GetDefaultTrainingConfig(TrainingConfig_t *config)
{
    if (config == 0) return;
    memset(config, 0, sizeof(*config));
    config->version = APP_CONFIG_VERSION;
    config->x_min = 80U; config->x_home = 90U; config->x_max = 110U;
    config->y_min = 85U; config->y_home = 90U; config->y_max = 95U;
    config->servo_settle_ms = 100U;
    config->fixation_duration_s = 15U;
    config->saccade_trials = 8U;
    config->saccade_response_ms = 3000U;
    config->saccade_interval_ms = 500U;
    config->saccade_pattern = APP_SACCADE_CORNERS;
    config->pursuit_speed_deg_s = 15U;
    config->pursuit_path = APP_PURSUIT_FULL;
    config->pursuit_rounds = 1U;
    config->pursuit_checkpoint_ms = 2000U;
    config->focus_trials = 5U;
    config->focus_timeout_s = 5U;
    config->focus_interval_s = 2U;
    config->focus_feedback_required = 1U;
    config->neglect_trials = 6U;
    config->neglect_timeout_ms = 5000U;
    config->neglect_affected_side = 0U;
    config->neglect_affected_ratio = 50U;
    config->neglect_pattern = APP_NEGLECT_ALTERNATE;
    config->head_warning_tenths = 50U;
    config->safety_turn_deg = 20U;
    config->safety_nod_deg = 20U;
    config->safety_tilt_deg = 30U;
    config->resume_stable_s = 3U;
    config->head_variation_limit_deg = 3U;
}

TrainingConfig_t App_GetTrainingConfig(void) { return training_config; }

AppControlResult_t App_SetTrainingConfig(const TrainingConfig_t *config)
{
    AppControlResult_t result = APP_CONTROL_RESULT_ACCEPTED;
    if (sys_state != SYS_IDLE_VOICE && sys_state != SYS_MODE_SELECT && sys_state != SYS_FEEDBACK)
        return APP_CONTROL_RESULT_INVALID_STATE;
    if (!App_ConfigIsValid(config, &result))
        return result;
    training_config = *config;
    /* 参数同步只更新下一次训练配置，不能在空闲时突然推动头戴云台。 */
    return APP_CONTROL_RESULT_ACCEPTED;
}

AppControlResult_t App_PreviewServo(uint8_t axis, uint8_t angle,
                                   AppServoPreviewAction_t action)
{
    if (sys_state != SYS_IDLE_VOICE && sys_state != SYS_MODE_SELECT &&
        sys_state != SYS_FEEDBACK)
        return APP_CONTROL_RESULT_INVALID_STATE;

    /* 云台维护期间绝不允许激光随舵机移动。 */
    App_LaserOff();
    App_LEDFocusOff();

    if (action == APP_SERVO_PREVIEW_PARK)
    {
        App_SetServoPark();
        return APP_CONTROL_RESULT_ACCEPTED;
    }
    if (action == APP_SERVO_PREVIEW_HOME)
    {
        App_SetServoCenter();
        return APP_CONTROL_RESULT_ACCEPTED;
    }
    if (action != APP_SERVO_PREVIEW_MOVE ||
        (axis != SERVO_AXIS_X && axis != SERVO_AXIS_Y))
        return APP_CONTROL_RESULT_INVALID_FIELD;
    if (angle < APP_SERVO_HARD_MIN || angle > APP_SERVO_HARD_MAX)
        return APP_CONTROL_RESULT_OUT_OF_RANGE;

    App_SetServoAngle(axis, angle);
    return APP_CONTROL_RESULT_ACCEPTED;
}

AppControlResult_t App_StartSession(const TrainingPlan_t *plan)
{
    if (sys_state != SYS_IDLE_VOICE && sys_state != SYS_MODE_SELECT && sys_state != SYS_FEEDBACK)
        return APP_CONTROL_RESULT_INVALID_STATE;
    if (!App_ValidatePlan(plan))
        return APP_CONTROL_RESULT_INVALID_ORDER;

    active_plan = *plan;
    if (active_plan.flow_mode == APP_FLOW_FULL)
    {
        active_plan.mode_count = MODE_COUNT;
        for (uint8_t i = 0U; i < MODE_COUNT; ++i) active_plan.mode_order[i] = i;
    }
    flow_mode = (AppFlowMode_t)active_plan.flow_mode;
    pending_flow_mode = flow_mode;
    active_plan_index = 0U;
    session_active = 1U;
    App_ResetSessionReport(&active_plan);
    App_StartPlanMode(0U);
    return APP_CONTROL_RESULT_ACCEPTED;
}

AppControlResult_t App_ExecuteControl(AppControlCommand_t command, uint8_t value)
{
    switch (command)
    {
        case APP_CONTROL_START:
        {
            TrainingPlan_t plan;
            if (value >= (uint8_t)MODE_COUNT)
                return APP_CONTROL_RESULT_INVALID_MODE;
            memset(&plan, 0, sizeof(plan));
            plan.flow_mode = APP_FLOW_CUSTOM;
            plan.mode_count = 1U;
            plan.mode_order[0] = value;
            return App_StartSession(&plan);
        }

        case APP_CONTROL_PAUSE:
            if (sys_state != SYS_TRAIN)
                return APP_CONTROL_RESULT_INVALID_STATE;
            App_EnterPause(0);
            pause_by_control = 1;
            return APP_CONTROL_RESULT_ACCEPTED;

        case APP_CONTROL_RESUME:
            /* 屏幕不能越过姿态安全条件，只有屏幕主动暂停才能由屏幕继续。 */
            if (sys_state != SYS_PAUSE || pause_by_control == 0U)
                return APP_CONTROL_RESULT_INVALID_STATE;
            pause_by_control = 0;
            App_ResumeFromPause();
            return APP_CONTROL_RESULT_ACCEPTED;

        case APP_CONTROL_STOP:
            App_StopFromControl();
            return APP_CONTROL_RESULT_ACCEPTED;

        case APP_CONTROL_CALIBRATE:
            if (sys_state != SYS_IDLE_VOICE && sys_state != SYS_MODE_SELECT &&
                sys_state != SYS_FEEDBACK)
                return APP_CONTROL_RESULT_INVALID_STATE;
            App_EnterServoCalibReady(0U);
            return APP_CONTROL_RESULT_ACCEPTED;

        default:
            return APP_CONTROL_RESULT_INVALID_STATE;
    }
}

uint8_t App_GetStatusFlags(void)
{
    uint8_t flags = 0U;

    if (sys_state == SYS_TRAIN)
        flags |= APP_STATUS_RUNNING;
    if (sys_state == SYS_PAUSE)
        flags |= APP_STATUS_PAUSED;
    if (sys_state == SYS_CALIBRATE || sys_state == SYS_NEXT_CONFIRM ||
        sys_state == SYS_SERVO_CALIB_READY || sys_state == SYS_CALIB_SERVO)
        flags |= APP_STATUS_WAITING_PA2;
    if (record.completed != 0U)
        flags |= APP_STATUS_MODE_COMPLETED;
    if (pause_by_control != 0U)
        flags |= APP_STATUS_CONTROL_PAUSE;
    if (session_report.completed != 0U)
        flags |= APP_STATUS_SESSION_COMPLETED;

    return flags;
}

void App_GetLiveStatus(AppLiveStatus_t *status)
{
    ModeReport_t *mode_report;
    uint32_t end_tick;
    if (status == 0) return;
    memset(status, 0, sizeof(*status));
    mode_report = &mode_reports[(uint8_t)train_mode];
    end_tick = (record.completed != 0U && record.end_tick != 0U) ? record.end_tick : HAL_GetTick();
    status->state = (uint8_t)sys_state;
    status->mode = (uint8_t)train_mode;
    status->flow_mode = (uint8_t)flow_mode;
    status->flags = App_GetStatusFlags();
    status->mode_index = active_plan_index;
    status->mode_count = active_plan.mode_count;
    status->total_trials = mode_report->total_trials;
    status->correct_trials = mode_report->correct_trials;
    status->timeout_trials = mode_report->timeout_trials;
    if (record.start_tick != 0U && end_tick >= record.start_tick)
        status->elapsed_ms = end_tick - record.start_tick;
    if (mode_report->correct_trials != 0U)
        status->avg_reaction_ms = (uint16_t)(mode_report->reaction_sum_ms /
                                             mode_report->correct_trials);
    status->head_offset_tenths = (uint16_t)(live_head_offset_deg * 10.0f + 0.5f);
    status->head_variation_tenths = (uint16_t)(HeadTracker_GetResult()->head_stability * 10.0f + 0.5f);
    status->session_id = session_report.session_id;
    if (sys_state == SYS_CALIB_SERVO)
    {
        status->calibration_phase = calib_phase;
        status->calibration_points = calib_point_count;
    }
}

TrainingSessionReport_t App_GetSessionReport(void) { return session_report; }

uint8_t App_GetModeReport(uint8_t mode, ModeReport_t *report)
{
    if (mode >= MODE_COUNT || report == 0) return 0U;
    *report = mode_reports[mode];
    return 1U;
}

uint8_t App_GetTrialResult(uint8_t mode, uint8_t index, TrialResult_t *result)
{
    if (mode >= MODE_COUNT || index >= trial_counts[mode] || result == 0) return 0U;
    *result = trial_results[mode][index];
    return 1U;
}

uint8_t App_GetTrialCount(uint8_t mode)
{
    return (mode < MODE_COUNT) ? trial_counts[mode] : 0U;
}
uint8_t App_GetLastVoiceCmd(void) { return last_voice_cmd; }
uint32_t App_GetLastVoiceCmdTick(void) { return last_voice_cmd_tick; }
uint32_t App_GetPauseEnterTick(void) { return pause_enter_tick; }
uint32_t App_GetLastPausedMs(void) { return paused_ms_last; }
uint8_t App_GetLastEvent(void) { return last_app_event; }

static uint8_t App_ConfigIsValid(const TrainingConfig_t *config,
                                 AppControlResult_t *result)
{
    if (config == 0)
    {
        *result = APP_CONTROL_RESULT_INVALID_FIELD;
        return 0U;
    }
    if (config->version != APP_CONFIG_VERSION)
    {
        *result = APP_CONTROL_RESULT_VERSION_MISMATCH;
        return 0U;
    }
    if (config->x_min < APP_SERVO_HARD_MIN || config->x_max > APP_SERVO_HARD_MAX ||
        config->x_min >= config->x_home || config->x_home >= config->x_max ||
        (uint8_t)(config->x_max - config->x_min) < 10U ||
        config->y_min < APP_SERVO_HARD_MIN || config->y_max > APP_SERVO_HARD_MAX ||
        config->y_min >= config->y_home || config->y_home >= config->y_max ||
        (uint8_t)(config->y_max - config->y_min) < 5U)
    {
        *result = APP_CONTROL_RESULT_OUT_OF_RANGE;
        return 0U;
    }
    if (config->servo_settle_ms < 100U || config->servo_settle_ms > 500U ||
        config->fixation_duration_s < 5U || config->fixation_duration_s > 60U ||
        config->saccade_trials < 4U || config->saccade_trials > 20U ||
        config->saccade_response_ms < 1000U || config->saccade_response_ms > 5000U ||
        config->saccade_interval_ms < 300U || config->saccade_interval_ms > 1000U ||
        config->saccade_pattern > APP_SACCADE_MIXED ||
        config->pursuit_speed_deg_s < 5U || config->pursuit_speed_deg_s > 30U ||
        config->pursuit_path > APP_PURSUIT_CROSS ||
        config->pursuit_rounds < 1U || config->pursuit_rounds > 3U ||
        config->pursuit_checkpoint_ms < 1000U || config->pursuit_checkpoint_ms > 5000U ||
        config->focus_trials < 2U || config->focus_trials > 10U ||
        config->focus_timeout_s < 2U || config->focus_timeout_s > 10U ||
        config->focus_interval_s < 1U || config->focus_interval_s > 10U ||
        config->focus_feedback_required != 1U ||
        config->neglect_trials < 4U || config->neglect_trials > 20U ||
        config->neglect_timeout_ms < 2000U || config->neglect_timeout_ms > 8000U ||
        config->neglect_affected_side > 1U ||
        config->neglect_affected_ratio < 50U || config->neglect_affected_ratio > 80U ||
        config->neglect_pattern > APP_NEGLECT_WEIGHTED_RANDOM)
    {
        *result = APP_CONTROL_RESULT_OUT_OF_RANGE;
        return 0U;
    }
    /* 高级页只能收紧安全条件，不能突破固件硬上限。 */
    if (config->head_warning_tenths < 30U || config->head_warning_tenths > 100U ||
        config->safety_turn_deg < 10U || config->safety_turn_deg > 20U ||
        config->safety_nod_deg < 10U || config->safety_nod_deg > 20U ||
        config->safety_tilt_deg < 15U || config->safety_tilt_deg > 30U ||
        config->resume_stable_s < 2U || config->resume_stable_s > 10U ||
        config->head_variation_limit_deg < 1U || config->head_variation_limit_deg > 3U)
    {
        *result = APP_CONTROL_RESULT_OUT_OF_RANGE;
        return 0U;
    }
    return 1U;
}

static uint8_t App_ValidatePlan(const TrainingPlan_t *plan)
{
    uint8_t used = 0U;
    if (plan == 0 || plan->flow_mode > APP_FLOW_CUSTOM) return 0U;
    if (plan->flow_mode == APP_FLOW_FULL) return 1U;
    if (plan->mode_count == 0U || plan->mode_count > APP_MAX_PLAN_MODES) return 0U;
    for (uint8_t i = 0U; i < plan->mode_count; ++i)
    {
        uint8_t mode = plan->mode_order[i];
        if (mode >= MODE_COUNT || (used & (uint8_t)(1U << mode)) != 0U) return 0U;
        used |= (uint8_t)(1U << mode);
    }
    return 1U;
}

static void App_ResetSessionReport(const TrainingPlan_t *plan)
{
    memset(&session_report, 0, sizeof(session_report));
    memset(mode_reports, 0, sizeof(mode_reports));
    memset(trial_results, 0, sizeof(trial_results));
    memset(trial_counts, 0, sizeof(trial_counts));
    session_report.session_id = next_session_id++;
    if (next_session_id == 0U) next_session_id = 1U;
    session_report.flow_mode = plan->flow_mode;
    session_report.mode_count = plan->mode_count;
    memcpy(session_report.mode_order, plan->mode_order, APP_MAX_PLAN_MODES);
}

static void App_ResetModeReport(TrainMode_t mode)
{
    ModeReport_t *report = &mode_reports[(uint8_t)mode];
    memset(report, 0, sizeof(*report));
    report->mode = (uint8_t)mode;
    report->min_reaction_ms = 0xFFFFFFFFU;
    memset(&mode_metrics, 0, sizeof(mode_metrics));
    memset(trial_results[(uint8_t)mode], 0, sizeof(trial_results[(uint8_t)mode]));
    trial_counts[(uint8_t)mode] = 0U;
}

static void App_StartPlanMode(uint8_t index)
{
    if (index >= active_plan.mode_count) return;
    active_plan_index = index;
    train_mode = (TrainMode_t)active_plan.mode_order[index];
    completed_modes = session_report.completed_modes;
    App_SelectModeAndCalibrate(train_mode);
}

static void App_UpdateModeMetrics(void)
{
    uint32_t now = HAL_GetTick();
    HeadAnalysis_t *head = HeadTracker_GetResult();
    if (mode_metrics.last_sample_tick != 0U && (now - mode_metrics.last_sample_tick) < 8U) return;
    mode_metrics.last_sample_tick = now;
    mode_metrics.sample_count++;
    mode_metrics.offset_sum_deg += live_head_offset_deg;
    mode_metrics.variation_sum_deg += head->head_stability;
    if (live_head_offset_deg * 10.0f > (float)training_config.head_warning_tenths)
        mode_reports[(uint8_t)train_mode].head_over_limit_ms += 10U;
    if (fabsf(head->pitch) > (float)training_config.safety_turn_deg)
        mode_metrics.compensatory_samples++;
    if ((uint16_t)(live_head_offset_deg * 10.0f + 0.5f) >
        mode_reports[(uint8_t)train_mode].max_head_offset_tenths)
        mode_reports[(uint8_t)train_mode].max_head_offset_tenths =
            (uint16_t)(live_head_offset_deg * 10.0f + 0.5f);
}

static void App_RecordTrial(uint8_t target, uint8_t side, uint8_t correct,
                            uint8_t timed_out, uint32_t reaction_ms)
{
    uint8_t mode = (uint8_t)train_mode;
    ModeReport_t *report = &mode_reports[mode];
    if (trial_counts[mode] < APP_MAX_TRIALS_PER_MODE)
    {
        TrialResult_t *trial = &trial_results[mode][trial_counts[mode]++];
        trial->target = target;
        trial->side = side;
        trial->correct = correct;
        trial->timed_out = timed_out;
        trial->reaction_ms = reaction_ms;
    }
    report->total_trials++;
    /* 安装语义：X_MAX/side=1为患者左侧，X_MIN/side=0为患者右侧。 */
    if (side != 0U) report->left_total++; else report->right_total++;
    if (correct != 0U)
    {
        report->correct_trials++;
        report->reaction_sum_ms += reaction_ms;
        if (side != 0U) report->left_correct++; else report->right_correct++;
        if (reaction_ms < report->min_reaction_ms) report->min_reaction_ms = reaction_ms;
        if (reaction_ms > report->max_reaction_ms) report->max_reaction_ms = reaction_ms;
    }
    if (timed_out != 0U) report->timeout_trials++;
    record.total_trials = report->total_trials;
    record.correct_trials = report->correct_trials;
    record.reaction_sum_ms = (float)report->reaction_sum_ms;
}

static uint8_t App_CalculateHeadScore(const ModeReport_t *report)
{
    uint16_t limit = (uint16_t)training_config.safety_turn_deg * 10U;
    if (limit == 0U || report->avg_head_offset_tenths >= limit) return 0U;
    return (uint8_t)(100U - ((uint32_t)report->avg_head_offset_tenths * 100U / limit));
}

static uint8_t App_CalculateReactionScore(const ModeReport_t *report)
{
    uint32_t timeout = training_config.saccade_response_ms;
    if (report->mode == MODE_C_PURSUIT) timeout = training_config.pursuit_checkpoint_ms;
    if (report->mode == MODE_D_FOCUS) timeout = (uint32_t)training_config.focus_timeout_s * 1000U;
    if (report->mode == MODE_E_NEGLECT) timeout = training_config.neglect_timeout_ms;
    uint32_t best = 500U;
    if (report->avg_reaction_ms <= best) return 100U;
    if (report->avg_reaction_ms >= timeout || timeout <= best) return 0U;
    return (uint8_t)(((timeout - report->avg_reaction_ms) * 100U) / (timeout - best));
}

static uint8_t App_CalculateModeScore(const ModeReport_t *report)
{
    uint8_t completion = report->completed ? 100U : 0U;
    uint8_t accuracy = report->total_trials ?
                       (uint8_t)((uint32_t)report->correct_trials * 100U / report->total_trials) : completion;
    uint8_t head_score = App_CalculateHeadScore(report);
    uint8_t reaction = App_CalculateReactionScore(report);
    uint8_t no_compensation = (uint8_t)(100U - (report->compensation_permille / 10U));
    switch ((TrainMode_t)report->mode)
    {
        case MODE_A_FIXATION: return (uint8_t)((completion * 30U + head_score * 70U) / 100U);
        case MODE_B_SACCADE: return (uint8_t)((accuracy * 50U + reaction * 30U + head_score * 20U) / 100U);
        case MODE_C_PURSUIT: return (uint8_t)((accuracy * 30U + no_compensation * 40U + head_score * 30U) / 100U);
        case MODE_D_FOCUS:
            return (uint8_t)((accuracy * 80U + head_score * 20U) / 100U);
        case MODE_E_NEGLECT:
        {
            uint16_t side_total = training_config.neglect_affected_side ? report->left_total : report->right_total;
            uint16_t side_correct = training_config.neglect_affected_side ? report->left_correct : report->right_correct;
            uint8_t side_accuracy = side_total ? (uint8_t)((uint32_t)side_correct * 100U / side_total) : 0U;
            return (uint8_t)((accuracy * 30U + side_accuracy * 25U + reaction * 25U + head_score * 20U) / 100U);
        }
        default: return 0U;
    }
}

static void App_FinalizeModeReport(void)
{
    ModeReport_t *report = &mode_reports[(uint8_t)train_mode];
    uint32_t score_sum = 0U;
    report->completed = 1U;
    report->duration_ms = record.end_tick - record.start_tick;
    if (report->correct_trials != 0U)
        report->avg_reaction_ms = report->reaction_sum_ms / report->correct_trials;
    if (report->min_reaction_ms == 0xFFFFFFFFU) report->min_reaction_ms = 0U;
    if (mode_metrics.sample_count != 0U)
    {
        report->avg_head_offset_tenths =
            (uint16_t)(mode_metrics.offset_sum_deg * 10.0f / mode_metrics.sample_count + 0.5f);
        report->head_variation_tenths =
            (uint16_t)(mode_metrics.variation_sum_deg * 10.0f / mode_metrics.sample_count + 0.5f);
        report->compensation_permille =
            (uint16_t)(mode_metrics.compensatory_samples * 1000U / mode_metrics.sample_count);
    }
    report->score = App_CalculateModeScore(report);
    record.avg_head_variation = report->head_variation_tenths / 10.0f;
    record.completed = 1U;
    session_report.completed_modes++;
    session_report.total_duration_ms += report->duration_ms;
    for (uint8_t i = 0U; i < session_report.mode_count; ++i)
        score_sum += mode_reports[session_report.mode_order[i]].score;
    if (session_report.completed_modes != 0U)
        session_report.overall_score = (uint8_t)(score_sum / session_report.completed_modes);
}

static void App_CompleteCurrentMode(void)
{
    App_StopStimulus();
    record.end_tick = HAL_GetTick();
    App_FinalizeModeReport();
    App_Transition(SYS_FEEDBACK);
}

static uint8_t App_IsSessionLastMode(void)
{
    return (uint8_t)(active_plan_index + 1U >= active_plan.mode_count);
}

static uint8_t App_NextSessionMode(void)
{
    if (App_IsSessionLastMode()) return 0U;
    active_plan_index++;
    train_mode = (TrainMode_t)active_plan.mode_order[active_plan_index];
    return 1U;
}

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
    mode_cmd_ack_tick = HAL_GetTick();
    App_Transition(SYS_MODE_CMD_ACK_WAIT);
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
        train_mode = MODE_A_FIXATION;

    /*
     * 进入完整/自选模式后暂不强制执行舵机视野标定。
     * 训练先使用 CALIB_X/Y_MIN/MAX 默认范围；需要标定时由语音“标定模式”触发。
     */

    if (flow_mode == APP_FLOW_FULL)
    {
        active_plan.flow_mode = APP_FLOW_FULL;
        active_plan.mode_count = MODE_COUNT;
        for (uint8_t i = 0U; i < MODE_COUNT; ++i) active_plan.mode_order[i] = i;
        active_plan_index = 0U;
        session_active = 1U;
        App_ResetSessionReport(&active_plan);
        train_mode = MODE_A_FIXATION;
        App_Transition(SYS_CALIBRATE);
    }
    else
    {
        App_Transition(SYS_IDLE_VOICE);
    }
}

static void App_State_ModeCmdAckWait(void)
{
    uint32_t now = HAL_GetTick();

    if (mode_cmd_ack_tick == 0U)
        mode_cmd_ack_tick = now;

    if ((now - mode_cmd_ack_tick) >= APP_MODE_CMD_ACK_WAIT_MS)
    {
        mode_cmd_ack_tick = 0;
        App_Transition(SYS_CALIBRATE);
    }
}
/**
 * @brief  校准状态
 */
static void App_State_ServoCalibReady(void)
{
    if (!servo_calib_prompted)
    {
        App_StopStimulus();
        (void)Key_GetEvent(KEY_PATIENT);
        Voice_Play(0xFF, VOICE_TTS_SERVO_CALIB_START);
        voice_cooldown = HAL_GetTick();
        servo_calib_prompted = 1;
        return;
    }

    if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT)
    {
        servo_calib_prompted = 0;
        calib_phase = 0;
        calib_pressed = 0;
        calib_save1 = 0;
        calib_save2 = 0;
        calib_point_count = 0;
        calib_x_min_pending = 0;
        calib_x_max_pending = 0;
        calib_y_min_pending = 0;
        calib_y_max_pending = 0;
        App_SetEvent(APP_EVENT_ENTER_SERVO_CALIB);
        sys_state = SYS_CALIB_SERVO;
    }
}

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
    uint32_t now = HAL_GetTick();
    ModeReport_t *report = &mode_reports[(uint8_t)train_mode];

    if (feedback_phase == 0U)
    {
        uint8_t rate = report->total_trials ?
                       (uint8_t)((uint32_t)report->correct_trials * 100U / report->total_trials) : 0U;
        switch (train_mode)
        {
            case MODE_A_FIXATION:
                if (report->avg_head_offset_tenths < 30U)
                    Voice_Play(0xFF, VOICE_TTS_FIX_GOOD);
                else if (report->avg_head_offset_tenths < 50U)
                    Voice_Play(0xFF, VOICE_TTS_FIX_FAIR);
                else
                    Voice_Play(0xFF, VOICE_TTS_FIX_POOR);
                break;
            case MODE_B_SACCADE:
                Voice_Play(0xFF, (rate >= 80U) ? VOICE_TTS_RESULT_GREAT :
                           ((rate >= 50U) ? VOICE_TTS_RESULT_GOOD : VOICE_TTS_RESULT_TRY));
                break;
            case MODE_C_PURSUIT:
                if (rate < 50U) Voice_Play(0xFF, VOICE_TTS_RESULT_TRY);
                else if (report->compensation_permille > 200U) Voice_Play(0xFF, VOICE_TTS_PUR_HEAD);
                else Voice_Play(0xFF, VOICE_TTS_PUR_GOOD);
                break;
            default:
                Voice_Play(0xFF, VOICE_TTS_TRAIN_DONE);
                break;
        }
        feedback_tick = now;
        feedback_phase = 1U;
        return;
    }

    if (feedback_phase == 1U && train_mode == MODE_B_SACCADE &&
        report->total_trials != 0U &&
        ((uint32_t)report->correct_trials * 100U / report->total_trials) >= 80U &&
        (now - feedback_tick) >= 2000U)
    {
        Voice_Play(0xFF, (report->avg_reaction_ms < 1500U) ?
                   VOICE_TTS_REACT_FAST : VOICE_TTS_REACT_SLOW);
        feedback_tick = now;
        feedback_phase = 2U;
        return;
    }

    if ((feedback_phase == 1U && (now - feedback_tick) < 8000U) ||
        (feedback_phase == 2U && (now - feedback_tick) < 6000U))
        return;

    if (feedback_phase == 3U)
    {
        if ((now - feedback_tick) >= 4000U) App_EnterModeSelect();
        return;
    }

    if (session_active != 0U && !App_IsSessionLastMode())
    {
        next_confirm_advance = 1U;
        feedback_phase = 0U;
        App_Transition(SYS_NEXT_CONFIRM);
        return;
    }

    session_report.completed = 1U;
    session_active = 0U;
    feedback_phase = 0U;
    if (flow_mode == APP_FLOW_FULL)
    {
        Voice_Play(0xFF, VOICE_TTS_ALL_DONE);
        feedback_tick = now;
        feedback_phase = 3U;
        return;
    }
    App_Transition(SYS_IDLE_VOICE);
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
        if (next_confirm_advance && session_active != 0U)
            (void)App_NextSessionMode();
        else if (next_confirm_advance)
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

    if (pause_by_voice || pause_by_control)
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
    if (fabsf(head->roll)  > (float)training_config.safety_nod_deg) alert = 1;
    if (fabsf(head->pitch) > (float)training_config.safety_turn_deg) alert = 1;
    if (fabsf(head->yaw)   > (float)training_config.safety_tilt_deg) alert = 1;

    if (!alert)
    {
        if (pause_stable_tick == 0)
            pause_stable_tick = now;

        if (now - pause_stable_tick >= (uint32_t)training_config.resume_stable_s * 1000U)
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
    if (fabsf(head->roll)  > (float)training_config.safety_nod_deg) alert = 1;
    if (fabsf(head->pitch) > (float)training_config.safety_turn_deg) alert = 1;
    if (fabsf(head->yaw)   > (float)training_config.safety_tilt_deg) alert = 1;

    if (alert)
    {
        App_EnterPause(0);
    }
}

static void App_SetServoAngle(uint8_t axis, uint8_t angle)
{
    if (angle < APP_SERVO_HARD_MIN) angle = APP_SERVO_HARD_MIN;
    if (angle > APP_SERVO_HARD_MAX) angle = APP_SERVO_HARD_MAX;
    Servo_SetAngle(axis, angle);
    if (axis == SERVO_AXIS_X)
        app_servo_x = angle;
    else if (axis == SERVO_AXIS_Y)
        app_servo_y = angle;
}

static uint8_t App_GetServoCenterX(void)
{
    return training_config.x_home;
}

static uint8_t App_GetServoCenterY(void)
{
    return training_config.y_home;
}

static void App_SetServoCenter(void)
{
    App_SetServoAngle(SERVO_AXIS_X, App_GetServoCenterX());
    App_SetServoAngle(SERVO_AXIS_Y, App_GetServoCenterY());
}

static void App_SetServoPark(void)
{
    App_SetServoAngle(SERVO_AXIS_X, APP_SERVO_PARK_ANGLE);
    App_SetServoAngle(SERVO_AXIS_Y, APP_SERVO_PARK_ANGLE);
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
    App_SetServoPark();
}

static void App_EnterPause(uint8_t play_posture_voice)
{
    if (sys_state == SYS_PAUSE)
        return;

    if (sys_state == SYS_TRAIN && mode_reports[(uint8_t)train_mode].pause_count < 255U)
        mode_reports[(uint8_t)train_mode].pause_count++;
    App_SetEvent(APP_EVENT_ENTER_PAUSE);
    pause_resume_x = app_servo_x;
    pause_resume_y = app_servo_y;
    pause_resume_laser = app_laser_on;
    pause_resume_led = app_led_focus_on;
    pause_enter_tick = HAL_GetTick();
    pause_stable_tick = 0;
    pause_voice_played = 0;
    pause_by_voice = 0;
    pause_by_control = 0;

    if (play_posture_voice)
    {
        Voice_Play(0xFF, VOICE_TTS_POSTURE);
        voice_cooldown = HAL_GetTick();
        Buzzer_Alert(2, 150, 100);
        pause_voice_played = 1;
    }

    App_LaserOff();
    App_LEDFocusOff();
    App_SetServoPark();
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
    pause_by_control = 0;
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
    pause_by_control = 0;
    voice_listen_active = 0;
    mode_select_prompted = 0;
    mode_select_clicks = 0;
    mode_select_first_tick = 0;
    next_confirm_prompted = 0;
    App_Transition(SYS_MODE_SELECT);
}

static void App_StopFromControl(void)
{
    App_StopStimulus();
    record.completed = 0U;
    record.end_tick = HAL_GetTick();
    pause_stable_tick = 0U;
    pause_voice_played = 0U;
    pause_enter_tick = 0U;
    pause_by_voice = 0U;
    pause_by_control = 0U;
    voice_listen_active = 0U;
    calib_prompted = 0U;
    train_prompt_played = 0U;
    mode_cmd_ack_tick = 0U;
    next_confirm_prompted = 0U;
    mode_select_prompted = 0U;
    session_active = 0U;
    session_report.completed = 0U;
    calib_phase = 0U;
    calib_point_count = 0U;
    calib_wait_until = 0U;
    App_Transition(SYS_IDLE_VOICE);
    App_SetEvent(APP_EVENT_CONTROL_STOP);
}

static void App_EnterServoCalibReady(uint8_t after_mode_enter)
{
    App_StopStimulus();
    record.completed = 0;
    pause_stable_tick = 0;
    pause_voice_played = 0;
    pause_enter_tick = 0;
    pause_by_voice = 0;
    pause_by_control = 0;
    voice_listen_active = 0;
    next_confirm_prompted = 0;
    train_prompt_played = 0;
    mode_cmd_ack_tick = 0;
    servo_calib_after_mode_enter = after_mode_enter;
    servo_calib_prompted = 0;
    calib_phase = 0;
    calib_wait_until = 0U;
    calib_pressed = 0;
    calib_save1 = 0;
    calib_save2 = 0;
    calib_point_count = 0;
    calib_x_min_pending = 0;
    calib_x_max_pending = 0;
    calib_y_min_pending = 0;
    calib_y_max_pending = 0;
    App_Transition(SYS_SERVO_CALIB_READY);
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
    pause_by_control = 0;
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
    pause_stable_tick = 0;
    pause_voice_played = 0;
    pause_enter_tick = 0;
    pause_by_voice = 0;
    pause_by_control = 0;
    voice_listen_active = 0;

    if (session_active != 0U && App_NextSessionMode())
    {
        next_confirm_advance = 0;
        App_Transition(SYS_NEXT_CONFIRM);
    }
    else
    {
        session_active = 0U;
        App_Transition(SYS_IDLE_VOICE);
    }
    App_SetEvent(APP_EVENT_SKIP);
}

static void App_SelectModeAndCalibrate(TrainMode_t mode)
{
    if ((uint8_t)mode >= (uint8_t)MODE_COUNT)
        return;

    App_StopStimulus();
    if (mode_reports[(uint8_t)train_mode].completed != 0U)
    {
        if (session_report.completed_modes != 0U) session_report.completed_modes--;
        if (session_report.total_duration_ms >= mode_reports[(uint8_t)train_mode].duration_ms)
            session_report.total_duration_ms -= mode_reports[(uint8_t)train_mode].duration_ms;
    }
    if (session_active == 0U || active_plan.mode_count == 0U ||
        active_plan.mode_order[active_plan_index] != (uint8_t)mode)
    {
        memset(&active_plan, 0, sizeof(active_plan));
        active_plan.flow_mode = APP_FLOW_CUSTOM;
        active_plan.mode_count = 1U;
        active_plan.mode_order[0] = (uint8_t)mode;
        active_plan_index = 0U;
        flow_mode = APP_FLOW_CUSTOM;
        session_active = 1U;
        App_ResetSessionReport(&active_plan);
    }
    train_mode = mode;
    record.completed = 0;
    record.end_tick = HAL_GetTick();
    pause_stable_tick = 0;
    pause_voice_played = 0;
    pause_enter_tick = 0;
    pause_by_voice = 0;
    pause_by_control = 0;
    voice_listen_active = 0;
    mode_cmd_ack_tick = HAL_GetTick();
    App_Transition(SYS_MODE_CMD_ACK_WAIT);
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

static float SmoothStep(float x)
{
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x * x * (3.0f - 2.0f * x);
}

static void Pursuit_SetTarget(uint8_t idx)
{
    uint8_t x_mid = App_GetServoCenterX();
    uint8_t y_mid = App_GetServoCenterY();
    uint8_t point = (uint8_t)(idx % Pursuit_GetBasePointCount());

    if (training_config.pursuit_path == APP_PURSUIT_HORIZONTAL)
    {
        pursuit_target_x = point ? training_config.x_max : training_config.x_min;
        pursuit_target_y = y_mid;
        return;
    }
    if (training_config.pursuit_path == APP_PURSUIT_VERTICAL)
    {
        pursuit_target_x = x_mid;
        pursuit_target_y = point ? training_config.y_max : training_config.y_min;
        return;
    }

    switch (point)
    {
        case 0: pursuit_target_x = training_config.x_min; pursuit_target_y = y_mid; break;
        case 1: pursuit_target_x = training_config.x_max; pursuit_target_y = y_mid; break;
        case 2: pursuit_target_x = x_mid; pursuit_target_y = training_config.y_max; break;
        case 3: pursuit_target_x = x_mid; pursuit_target_y = training_config.y_min; break;
        case 4: pursuit_target_x = training_config.x_min; pursuit_target_y = training_config.y_max; break;
        case 5: pursuit_target_x = training_config.x_max; pursuit_target_y = training_config.y_min; break;
        case 6: pursuit_target_x = training_config.x_max; pursuit_target_y = training_config.y_max; break;
        default: pursuit_target_x = training_config.x_min; pursuit_target_y = training_config.y_min; break;
    }
}

static uint8_t Pursuit_GetBasePointCount(void)
{
    if (training_config.pursuit_path == APP_PURSUIT_HORIZONTAL ||
        training_config.pursuit_path == APP_PURSUIT_VERTICAL) return 2U;
    if (training_config.pursuit_path == APP_PURSUIT_CROSS) return 4U;
    return PURSUIT_POINT_COUNT;
}

static uint32_t Pursuit_CalculateMoveTime(void)
{
    float dx = (float)pursuit_target_x - (float)pursuit_start_x;
    float dy = (float)pursuit_target_y - (float)pursuit_start_y;
    uint32_t duration = (uint32_t)(sqrtf(dx * dx + dy * dy) * 1000.0f /
                                   training_config.pursuit_speed_deg_s);
    return (duration < 300U) ? 300U : duration;
}

static void Pursuit_Reset(void)
{
    pursuit_state = PURSUIT_MOVE;
    pursuit_point_idx = 0;
    pursuit_state_tick = HAL_GetTick();
    pursuit_start_x = App_GetServoCenterX();
    pursuit_start_y = App_GetServoCenterY();
    pursuit_total_points = (uint8_t)(Pursuit_GetBasePointCount() * training_config.pursuit_rounds);
    Pursuit_SetTarget(0);
    pursuit_move_time_ms = Pursuit_CalculateMoveTime();
}

static void Pursuit_AdvancePoint(uint32_t now)
{
    pursuit_point_idx++;
    if (pursuit_point_idx >= pursuit_total_points)
    {
        App_CompleteCurrentMode();
        return;
    }

    pursuit_start_x = pursuit_target_x;
    pursuit_start_y = pursuit_target_y;
    Pursuit_SetTarget(pursuit_point_idx);
    pursuit_move_time_ms = Pursuit_CalculateMoveTime();
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
        pause_by_control = 0;
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
        App_ResetModeReport(train_mode);
        record.mode = train_mode;
        record.start_tick = HAL_GetTick();
        timebase = HAL_GetTick();
        train_entry_tick = record.start_tick;
        Pursuit_Reset();
        focus_phase = 0;
        focus_target_active = 0U;
        focus_phase_tick = 0U;
        saccade_idx = 0;
        saccade_count = 0;
        saccade_phase = 0U;
        saccade_phase_tick = 0U;
        neglect_trial_count = 0;
        neglect_trial_tick = 0;
        neglect_phase = 0U;
        neglect_phase_tick = 0U;
        pause_stable_tick = 0;
        pause_voice_played = 0;
        voice_cooldown = 0;
        feedback_phase = 0U;
        feedback_tick = 0U;

        /* 模式播报由模块自动响应语音命令词完成，MCU 只需等待即可 */
        saccade_light_on_tick = 0;
    }

    if (next_state == SYS_FEEDBACK)
    {
        App_SetEvent(APP_EVENT_ENTER_FEEDBACK);
        feedback_phase = 0U;
        feedback_tick = 0U;
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
        mode_cmd_ack_tick = 0;
    }

    if (next_state == SYS_MODE_CMD_ACK_WAIT)
    {
        App_SetEvent(APP_EVENT_MODE_CMD_ACK_WAIT);
        mode_cmd_ack_tick = HAL_GetTick();
    }

    if (next_state == SYS_SERVO_CALIB_READY)
    {
        App_SetEvent(APP_EVENT_ENTER_SERVO_CALIB);
        servo_calib_prompted = 0;
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
    if (head->head_stability > (float)training_config.head_variation_limit_deg)
    {
        if (now - voice_cooldown > 3000U)
        {
            Voice_Play(0xFF, VOICE_TTS_KEEP_STILL);
            voice_cooldown = now;
        }
    }

    if (elapsed >= (uint32_t)training_config.fixation_duration_s * 1000U)
    {
        App_CompleteCurrentMode();
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
        saccade_count = training_config.saccade_trials;
        for (uint8_t i = 0; i < saccade_count; i++)
        {
            if (training_config.saccade_pattern == APP_SACCADE_HORIZONTAL)
                saccade_seq[i] = (uint8_t)(4U + (i & 1U));
            else if (training_config.saccade_pattern == APP_SACCADE_VERTICAL)
                saccade_seq[i] = (uint8_t)(6U + (i & 1U));
            else if (training_config.saccade_pattern == APP_SACCADE_MIXED)
                saccade_seq[i] = (uint8_t)(i % 8U);
            else
                saccade_seq[i] = (uint8_t)(i % 4U);
        }
        for (uint8_t i = saccade_count - 1; i > 0; i--)
        {
            uint8_t j = (uint8_t)((HAL_GetTick() + i * 17U) % (i + 1U));
            uint8_t tmp = saccade_seq[i];
            saccade_seq[i] = saccade_seq[j];
            saccade_seq[j] = tmp;
        }
        saccade_idx = 0;
        saccade_phase = 0U;
    }

    if (saccade_phase == 0U)
    {
        if (saccade_idx >= saccade_count)
        {
            App_CompleteCurrentMode();
            return;
        }

        current_target = saccade_seq[saccade_idx];
        uint8_t x_angle = training_config.x_home;
        uint8_t y_angle = training_config.y_home;
        switch (current_target)
        {
            case 0: x_angle = training_config.x_min; y_angle = training_config.y_min; break;
            case 1: x_angle = training_config.x_max; y_angle = training_config.y_min; break;
            case 2: x_angle = training_config.x_min; y_angle = training_config.y_max; break;
            case 3: x_angle = training_config.x_max; y_angle = training_config.y_max; break;
            case 4: x_angle = training_config.x_min; break;
            case 5: x_angle = training_config.x_max; break;
            case 6: y_angle = training_config.y_min; break;
            default: y_angle = training_config.y_max; break;
        }

        App_LaserOff();
        App_SetServoAngle(SERVO_AXIS_X, x_angle);
        App_SetServoAngle(SERVO_AXIS_Y, y_angle);
        saccade_phase_tick = now;
        saccade_phase = 1U;
        return;
    }

    if (saccade_phase == 1U)
    {
        if ((now - saccade_phase_tick) < training_config.servo_settle_ms) return;
        App_LaserOn();
        saccade_light_on_tick = now;
        trial_start_tick = now;
        trial_result = 0;
        saccade_phase = 2U;
        return;
    }

    if (saccade_phase == 2U && Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT)
    {
        uint8_t side = (current_target == 1U || current_target == 3U || current_target == 5U) ? 1U : 0U;
        App_RecordTrial(current_target, side, 1U, 0U, now - trial_start_tick);
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
        saccade_light_on_tick = 0;
        saccade_idx++;
        saccade_phase_tick = now;
        saccade_phase = 3U;
        return;
    }

    if (saccade_phase == 2U && (now - saccade_light_on_tick) >= training_config.saccade_response_ms)
    {
        uint8_t side = (current_target == 1U || current_target == 3U || current_target == 5U) ? 1U : 0U;
        App_RecordTrial(current_target, side, 0U, 1U, training_config.saccade_response_ms);
        saccade_streak = 0;
        App_LaserOff();
        Voice_Play(0xFF, VOICE_TTS_TIMEOUT);
        voice_cooldown = HAL_GetTick();
        saccade_light_on_tick = 0;
        saccade_idx++;
        saccade_phase_tick = now;
        saccade_phase = 3U;
        return;
    }

    if (saccade_phase == 3U && (now - saccade_phase_tick) >= training_config.saccade_interval_ms)
        saccade_phase = 0U;
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

    if (pursuit_state == PURSUIT_MOVE)
    {
        float s = (float)elapsed / (float)pursuit_move_time_ms;
        if (s > 1.0f) s = 1.0f;
        s = SmoothStep(s);

        uint8_t x_angle = (uint8_t)((float)pursuit_start_x +
                           ((float)pursuit_target_x - (float)pursuit_start_x) * s);
        uint8_t y_angle = (uint8_t)((float)pursuit_start_y +
                           ((float)pursuit_target_y - (float)pursuit_start_y) * s);
        App_SetServoAngle(SERVO_AXIS_X, x_angle);
        App_SetServoAngle(SERVO_AXIS_Y, y_angle);

        if (elapsed >= pursuit_move_time_ms)
        {
            App_SetServoAngle(SERVO_AXIS_X, pursuit_target_x);
            App_SetServoAngle(SERVO_AXIS_Y, pursuit_target_y);
            pursuit_state = PURSUIT_CHECKPOINT;
            pursuit_state_tick = now;
            trial_start_tick = now;
            trial_result = 0;
            Voice_Play(0xFF, VOICE_TTS_FIND_LIGHT);
            voice_cooldown = HAL_GetTick();
        }
        return;
    }

    if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && trial_result == 0)
    {
        uint8_t side = (pursuit_target_x >= training_config.x_home) ? 1U : 0U;
        trial_result = 1;
        App_RecordTrial(pursuit_point_idx, side, 1U, 0U, now - trial_start_tick);
        Voice_Play(0xFF, VOICE_TTS_CORRECT);
        voice_cooldown = HAL_GetTick();
        Pursuit_AdvancePoint(now);
        return;
    }

    if (elapsed >= training_config.pursuit_checkpoint_ms && trial_result == 0)
    {
        uint8_t side = (pursuit_target_x >= training_config.x_home) ? 1U : 0U;
        App_RecordTrial(pursuit_point_idx, side, 0U, 1U, training_config.pursuit_checkpoint_ms);
        Voice_Play(0xFF, VOICE_TTS_TIMEOUT);
        voice_cooldown = HAL_GetTick();
        Pursuit_AdvancePoint(now);
    }
}

/**
 * @brief  D — 单激光目标确认训练
 *         激光点亮后等待PA2反馈，记录反应时间或超时。
 */
static void Train_Focus(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t timeout_ms = (uint32_t)training_config.focus_timeout_s * 1000U;
    uint32_t interval_ms = (uint32_t)training_config.focus_interval_s * 1000U;

    if (focus_phase >= training_config.focus_trials)
    {
        App_CompleteCurrentMode();
        return;
    }

    if (focus_target_active == 0U)
    {
        App_LEDFocusOff();
        App_SetServoCenter();
        App_LaserOn();
        focus_phase_tick = now;
        focus_target_active = 1U;
        return;
    }

    if (focus_target_active == 1U && Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT)
    {
        App_RecordTrial(focus_phase, 0U, 1U, 0U, now - focus_phase_tick);
        App_LaserOff();
        focus_phase++;
        focus_target_active = 2U;
        focus_phase_tick = now;
        return;
    }

    if (focus_target_active == 1U && (now - focus_phase_tick) >= timeout_ms)
    {
        App_RecordTrial(focus_phase, 0U, 0U, 1U, timeout_ms);
        App_LaserOff();
        focus_phase++;
        focus_target_active = 2U;
        focus_phase_tick = now;
        return;
    }

    if (focus_target_active == 2U && (now - focus_phase_tick) >= interval_ms)
        focus_target_active = 0U;
}

/**
 * @brief  E — 空间忽略训练
 *         左右视野交替点亮激光，患者按 PA2 确认
 */
static void Train_Neglect(void)
{
    uint32_t now = HAL_GetTick();

    if (neglect_phase == 0U)
    {
        if (neglect_trial_count >= training_config.neglect_trials)
        {
            App_CompleteCurrentMode();
            return;
        }
        if (training_config.neglect_pattern == APP_NEGLECT_WEIGHTED_RANDOM)
        {
            uint8_t draw = (uint8_t)((now + neglect_trial_count * 37U) % 100U);
            neglect_side = (draw < training_config.neglect_affected_ratio) ?
                           training_config.neglect_affected_side :
                           (uint8_t)(1U - training_config.neglect_affected_side);
        }
        else
        {
            neglect_side = neglect_trial_count & 1U;
        }

        App_LaserOff();
        App_SetServoAngle(SERVO_AXIS_X,
                          neglect_side ? training_config.x_max : training_config.x_min);
        App_SetServoAngle(SERVO_AXIS_Y, training_config.y_home);
        neglect_phase_tick = now;
        neglect_phase = 1U;
        return;
    }

    if (neglect_phase == 1U)
    {
        if ((now - neglect_phase_tick) < training_config.servo_settle_ms) return;
        App_LaserOn();
        Voice_Play(0xFF, neglect_side ? VOICE_TTS_LEFT_SIDE : VOICE_TTS_RIGHT_SIDE);
        voice_cooldown = now;
        neglect_trial_tick = now;
        neglect_responded = 0;
        neglect_trial_count++;
        neglect_phase = 2U;
        return;
    }

    uint32_t reaction = now - neglect_trial_tick;

    if (neglect_phase == 2U && Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT)
    {
        neglect_responded = 1;
        App_RecordTrial((uint8_t)(neglect_trial_count - 1U), neglect_side, 1U, 0U, reaction);
        App_LaserOff();
        App_SetServoAngle(SERVO_AXIS_X, App_GetServoCenterX());
        Voice_Play(0xFF, VOICE_TTS_FOUND_SIDE);
        voice_cooldown = HAL_GetTick();
        neglect_trial_tick = 0;
        neglect_phase_tick = now;
        neglect_phase = 3U;
        return;
    }

    if (neglect_phase == 2U && reaction >= training_config.neglect_timeout_ms)
    {
        App_RecordTrial((uint8_t)(neglect_trial_count - 1U), neglect_side, 0U, 1U,
                        training_config.neglect_timeout_ms);
        Voice_Play(0xFF, VOICE_TTS_NEGLECT_HINT);
        voice_cooldown = HAL_GetTick();
        App_LaserOff();
        App_SetServoAngle(SERVO_AXIS_X, App_GetServoCenterX());
        neglect_trial_tick = 0;
        neglect_phase_tick = now;
        neglect_phase = 3U;
        return;
    }

    if (neglect_phase == 3U && (now - neglect_phase_tick) >= 1000U)
        neglect_phase = 0U;
}

/**
 * @brief  标定模式 — 按键标记视野边界
 *         语音命令「标定模式」触发，先标X轴（从30°向右扫描→按PA2标记边界）
 *         再标Y轴（向上扫描→按PA2标记边界），最后回到IDLE_VOICE
 *
 *         标定结果直接修改 CALIB_X/Y_MIN/MAX，训练模式自动使用新值。
 *         不重启则不丢，重启后还原为代码默认值。
 *
 *  相序:  0=开始语音 → 1=X右边界 → 2=X左边界 → 3=Y底边 → 4=Y顶边 → 5=完成
 */
static void State_CalibServo(void)
{
    uint8_t min_val;
    uint8_t max_val;
    uint32_t now = HAL_GetTick();

    /* 标定提示等待不阻塞主循环，USART10状态和停止命令仍可及时处理。 */
    if (calib_wait_until != 0U && (int32_t)(now - calib_wait_until) < 0)
        return;
    calib_wait_until = 0U;

    switch (calib_phase)
    {
        case 0:
            Buzzer_Alert(2, 150, 100);
            App_LaserOn();
            App_SetServoAngle(SERVO_AXIS_Y, APP_SERVO_PARK_ANGLE);
            App_SetServoAngle(SERVO_AXIS_X, 30);
            Voice_Play(0xFF, VOICE_TTS_CALIB_PRESS_POINT);
            voice_cooldown = now;
            calib_angle   = 30;
            calib_pressed = 0;
            calib_save1   = 0;
            calib_save2   = 0;
            calib_point_count = 0;
            calib_x_min_pending = 0;
            calib_x_max_pending = 0;
            calib_y_min_pending = 0;
            calib_y_max_pending = 0;
            calib_tick    = now;
            calib_phase   = 1;
            calib_wait_until = now + 2000U;
            break;

        /* ===== X轴标定（30°→150°→30°）===== */
        case 1:
            if (now - calib_tick < 100U) return;
            calib_tick = now;
            if (calib_angle < 150) calib_angle++;
            App_SetServoAngle(SERVO_AXIS_X, calib_angle);

            if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && calib_pressed == 0)
            {
                calib_save1   = calib_angle;
                calib_pressed = 1;
                calib_point_count++;
                Buzzer_Alert(1, 100, 0);
            }
            if (calib_angle >= 150)
            {
                calib_phase   = 2;
                calib_angle   = 150;
                calib_pressed = 0;
                Voice_Play(0xFF, VOICE_TTS_CALIB_PRESS_AGAIN);
                voice_cooldown = now;
                calib_wait_until = now + 1200U;
                calib_tick = now;
            }
            break;

        case 2:
            if (now - calib_tick < 100U) return;
            calib_tick = now;
            if (calib_angle > 30) calib_angle--;
            App_SetServoAngle(SERVO_AXIS_X, calib_angle);

            if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && calib_pressed == 0)
            {
                calib_save2   = calib_angle;
                calib_pressed = 1;
                calib_point_count++;
                Buzzer_Alert(1, 100, 0);
            }
            if (calib_angle <= 30)
            {
                if (calib_save1 != 0 && calib_save2 != 0)
                {
                    min_val = (calib_save1 < calib_save2) ? calib_save1 : calib_save2;
                    max_val = (calib_save1 > calib_save2) ? calib_save1 : calib_save2;
                    calib_x_min_pending = min_val;
                    calib_x_max_pending = max_val;
                    App_SetServoAngle(SERVO_AXIS_X, (uint8_t)(((uint16_t)min_val + (uint16_t)max_val) / 2U));
                }
                else
                {
                    App_SetServoAngle(SERVO_AXIS_X, App_GetServoCenterX());
                }

                calib_phase   = 3;
                calib_angle   = 75;
                App_SetServoAngle(SERVO_AXIS_Y, 75U);
                calib_pressed = 0;
                calib_save1   = 0;
                calib_save2   = 0;
                Buzzer_Alert(2, 150, 100);
                Voice_Play(0xFF, VOICE_TTS_CALIB_PRESS_POINT);
                voice_cooldown = now;
                calib_wait_until = now + 1500U;
                calib_tick = now;
            }
            break;

        /* ===== Y轴标定（80°→140°→80°）===== */
        case 3:
            if (now - calib_tick < 100U) return;
            calib_tick = now;
            if (calib_angle < 135) calib_angle++;
            App_SetServoAngle(SERVO_AXIS_Y, calib_angle);

            if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && calib_pressed == 0)
            {
                calib_save1   = calib_angle;
                calib_pressed = 1;
                calib_point_count++;
                Buzzer_Alert(1, 100, 0);
            }
            if (calib_angle >= 135)
            {
                calib_phase   = 4;
                calib_angle   = 135;
                calib_pressed = 0;
                Voice_Play(0xFF, VOICE_TTS_CALIB_PRESS_AGAIN);
                voice_cooldown = now;
                calib_wait_until = now + 1200U;
                calib_tick = now;
            }
            break;

        case 4:
            if (now - calib_tick < 100U) return;
            calib_tick = now;
            if (calib_angle > 75) calib_angle--;
            App_SetServoAngle(SERVO_AXIS_Y, calib_angle);

            if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT && calib_pressed == 0)
            {
                calib_save2   = calib_angle;
                calib_pressed = 1;
                calib_point_count++;
                Buzzer_Alert(1, 100, 0);
            }
            if (calib_angle <= 75)
            {
                if (calib_save1 != 0 && calib_save2 != 0)
                {
                    min_val = (calib_save1 < calib_save2) ? calib_save1 : calib_save2;
                    max_val = (calib_save1 > calib_save2) ? calib_save1 : calib_save2;
                    calib_y_min_pending = min_val;
                    calib_y_max_pending = max_val;
                }

                calib_phase = 5;
            }
            break;

        /* ===== 完成 ===== */
        case 5:
            if (calib_point_count < 4 ||
                calib_x_min_pending == 0 || calib_x_max_pending == 0 ||
                calib_y_min_pending == 0 || calib_y_max_pending == 0 ||
                (uint8_t)(calib_x_max_pending - calib_x_min_pending) < 10U ||
                (uint8_t)(calib_y_max_pending - calib_y_min_pending) < 5U)
            {
                App_LaserOff();
                Voice_Play(0xFF, VOICE_TTS_CALIB_POINTS_LOW);
                voice_cooldown = now;
                calib_wait_until = now + 2500U;
                calib_phase   = 0;
                calib_pressed = 0;
                calib_save1   = 0;
                calib_save2   = 0;
                calib_point_count = 0;
                calib_x_min_pending = 0;
                calib_x_max_pending = 0;
                calib_y_min_pending = 0;
                calib_y_max_pending = 0;
                break;
            }

            CALIB_X_MIN = calib_x_min_pending;
            CALIB_X_MAX = calib_x_max_pending;
            CALIB_Y_MIN = calib_y_min_pending;
            CALIB_Y_MAX = calib_y_max_pending;
            training_config.x_min = CALIB_X_MIN;
            training_config.x_max = CALIB_X_MAX;
            training_config.x_home = (uint8_t)(((uint16_t)CALIB_X_MIN + CALIB_X_MAX) / 2U);
            training_config.y_min = CALIB_Y_MIN;
            training_config.y_max = CALIB_Y_MAX;
            training_config.y_home = (uint8_t)(((uint16_t)CALIB_Y_MIN + CALIB_Y_MAX) / 2U);

            App_LaserOff();
            App_SetServoPark();
            Voice_Play(0xFF, VOICE_TTS_INIT_OK);
            voice_cooldown = now;
            calib_phase   = 6;
            calib_wait_until = now + 2500U;
            calib_pressed = 0;
            servo_range_calibrated = 1;
            App_SetEvent(APP_EVENT_SERVO_CALIB_DONE);
            break;

        case 6:
            calib_phase = 0;
            calib_point_count = 0;
            if (servo_calib_after_mode_enter)
            {
                servo_calib_after_mode_enter = 0;
                if (flow_mode == APP_FLOW_FULL)
                    App_Transition(SYS_CALIBRATE);
                else
                    App_Transition(SYS_IDLE_VOICE);
            }
            else
            {
                App_EnterModeSelect();
            }
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
