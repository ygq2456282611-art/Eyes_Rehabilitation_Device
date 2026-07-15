/**
 * @file train_modes.h
 * @brief 眼功能训练状态机、可调参数和训练报告公共接口。
 */
#ifndef TRAIN_MODES_H
#define TRAIN_MODES_H

#include "main.h"
#include "BMI088driver.h"

#define APP_CONFIG_VERSION          2U
#define APP_MAX_PLAN_MODES          5U
#define APP_MAX_TRIALS_PER_MODE    24U
#define APP_SERVO_HARD_MIN          10U
#define APP_SERVO_HARD_MAX         170U
#define APP_SERVO_PARK_ANGLE        90U

typedef enum {
    MODE_A_FIXATION = 0,
    MODE_B_SACCADE  = 1,
    MODE_C_PURSUIT  = 2,
    MODE_D_FOCUS    = 3,
    MODE_E_NEGLECT  = 4,
    MODE_COUNT      = 5
} TrainMode_t;

typedef enum {
    SYS_IDLE_VOICE = 0,
    SYS_CALIBRATE = 1,
    SYS_TRAIN = 2,
    SYS_FEEDBACK = 3,
    SYS_PAUSE = 4,
    SYS_CALIB_SERVO = 5,
    SYS_MODE_SELECT = 6,
    SYS_TRAIN_PROMPT = 7,
    SYS_NEXT_CONFIRM = 8,
    SYS_MODE_ENTER_PROMPT = 9,
    SYS_MODE_CMD_ACK_WAIT = 10,
    SYS_SERVO_CALIB_READY = 11
} SystemState_t;

typedef enum {
    APP_FLOW_FULL = 0,
    APP_FLOW_CUSTOM = 1
} AppFlowMode_t;

typedef enum {
    APP_CONTROL_START = 1,
    APP_CONTROL_PAUSE = 2,
    APP_CONTROL_RESUME = 3,
    APP_CONTROL_STOP = 4,
    APP_CONTROL_CALIBRATE = 5
} AppControlCommand_t;

typedef enum {
    APP_CONTROL_RESULT_ACCEPTED = 0,
    APP_CONTROL_RESULT_DUPLICATE = 1,
    APP_CONTROL_RESULT_INVALID_STATE = 2,
    APP_CONTROL_RESULT_INVALID_MODE = 3,
    APP_CONTROL_RESULT_QUEUE_BUSY = 4,
    APP_CONTROL_RESULT_INVALID_FIELD = 5,
    APP_CONTROL_RESULT_OUT_OF_RANGE = 6,
    APP_CONTROL_RESULT_INVALID_ORDER = 7,
    APP_CONTROL_RESULT_VERSION_MISMATCH = 8
} AppControlResult_t;

typedef enum {
    APP_SERVO_PREVIEW_MOVE = 0,
    APP_SERVO_PREVIEW_PARK = 1,
    APP_SERVO_PREVIEW_HOME = 2
} AppServoPreviewAction_t;

typedef enum {
    APP_SACCADE_CORNERS = 0,
    APP_SACCADE_HORIZONTAL = 1,
    APP_SACCADE_VERTICAL = 2,
    APP_SACCADE_MIXED = 3
} AppSaccadePattern_t;

typedef enum {
    APP_PURSUIT_FULL = 0,
    APP_PURSUIT_HORIZONTAL = 1,
    APP_PURSUIT_VERTICAL = 2,
    APP_PURSUIT_CROSS = 3
} AppPursuitPath_t;

typedef enum {
    APP_NEGLECT_ALTERNATE = 0,
    APP_NEGLECT_WEIGHTED_RANDOM = 1
} AppNeglectPattern_t;

#define APP_STATUS_RUNNING           (1U << 0)
#define APP_STATUS_PAUSED            (1U << 1)
#define APP_STATUS_WAITING_PA2       (1U << 2)
#define APP_STATUS_MODE_COMPLETED    (1U << 3)
#define APP_STATUS_CONTROL_PAUSE     (1U << 4)
#define APP_STATUS_SESSION_COMPLETED (1U << 5)

typedef struct {
    uint8_t version;
    uint8_t x_min;
    uint8_t x_home;
    uint8_t x_max;
    uint8_t y_min;
    uint8_t y_home;
    uint8_t y_max;
    uint16_t servo_settle_ms;
    uint8_t fixation_duration_s;
    uint8_t saccade_trials;
    uint16_t saccade_response_ms;
    uint16_t saccade_interval_ms;
    uint8_t saccade_pattern;
    uint8_t pursuit_speed_deg_s;
    uint8_t pursuit_path;
    uint8_t pursuit_rounds;
    uint16_t pursuit_checkpoint_ms;
    uint8_t focus_trials;
    uint8_t focus_timeout_s;
    uint8_t focus_interval_s;
    uint8_t focus_feedback_required;
    uint8_t neglect_trials;
    uint16_t neglect_timeout_ms;
    uint8_t neglect_affected_side;
    uint8_t neglect_affected_ratio;
    uint8_t neglect_pattern;
    uint16_t head_warning_tenths;
    uint8_t safety_turn_deg;
    uint8_t safety_nod_deg;
    uint8_t safety_tilt_deg;
    uint8_t resume_stable_s;
    uint8_t head_variation_limit_deg;
} TrainingConfig_t;

typedef struct {
    uint8_t flow_mode;
    uint8_t mode_count;
    uint8_t mode_order[APP_MAX_PLAN_MODES];
} TrainingPlan_t;

typedef struct {
    uint8_t target;
    uint8_t side;
    uint8_t correct;
    uint8_t timed_out;
    uint32_t reaction_ms;
} TrialResult_t;

typedef struct {
    uint8_t mode;
    uint8_t completed;
    uint8_t score;
    uint8_t pause_count;
    uint16_t total_trials;
    uint16_t correct_trials;
    uint16_t timeout_trials;
    uint16_t left_total;
    uint16_t left_correct;
    uint16_t right_total;
    uint16_t right_correct;
    uint32_t duration_ms;
    uint32_t reaction_sum_ms;
    uint32_t avg_reaction_ms;
    uint32_t min_reaction_ms;
    uint32_t max_reaction_ms;
    uint16_t avg_head_offset_tenths;
    uint16_t max_head_offset_tenths;
    uint16_t head_variation_tenths;
    uint16_t compensation_permille;
    uint32_t head_over_limit_ms;
} ModeReport_t;

typedef struct {
    uint32_t session_id;
    uint8_t flow_mode;
    uint8_t mode_count;
    uint8_t mode_order[APP_MAX_PLAN_MODES];
    uint8_t completed_modes;
    uint8_t completed;
    uint8_t overall_score;
    uint32_t total_duration_ms;
} TrainingSessionReport_t;

/* 兼容现有调试和语音反馈代码的当前模式摘要。 */
typedef struct {
    TrainMode_t mode;
    uint32_t start_tick;
    uint32_t end_tick;
    uint16_t total_trials;
    uint16_t correct_trials;
    float reaction_sum_ms;
    float avg_head_variation;
    uint8_t completed;
} TrainingRecord_t;

typedef struct {
    uint8_t state;
    uint8_t mode;
    uint8_t flow_mode;
    uint8_t flags;
    uint8_t mode_index;
    uint8_t mode_count;
    uint16_t total_trials;
    uint16_t correct_trials;
    uint16_t timeout_trials;
    uint32_t elapsed_ms;
    uint16_t avg_reaction_ms;
    uint16_t head_offset_tenths;
    uint16_t head_variation_tenths;
    uint32_t session_id;
    uint8_t calibration_phase;
    uint8_t calibration_points;
} AppLiveStatus_t;

void App_Init(void);
void App_Run(bmi088_euler_data_t *euler, float temp);
SystemState_t App_GetState(void);
TrainMode_t App_GetMode(void);
TrainingRecord_t App_GetRecord(void);

/** 在空闲状态校验并应用下一次训练使用的配置。 */
AppControlResult_t App_SetTrainingConfig(const TrainingConfig_t *config);
void App_GetDefaultTrainingConfig(TrainingConfig_t *config);
TrainingConfig_t App_GetTrainingConfig(void);

/** 空闲维护时预览舵机；预览会强制关闭激光，只接受10到170度。 */
AppControlResult_t App_PreviewServo(uint8_t axis, uint8_t angle,
                                   AppServoPreviewAction_t action);

/** 启动完整或自选训练；自选顺序不得重复。 */
AppControlResult_t App_StartSession(const TrainingPlan_t *plan);
AppControlResult_t App_ExecuteControl(AppControlCommand_t command, uint8_t value);
void App_GetLiveStatus(AppLiveStatus_t *status);
TrainingSessionReport_t App_GetSessionReport(void);
uint8_t App_GetModeReport(uint8_t mode, ModeReport_t *report);
uint8_t App_GetTrialResult(uint8_t mode, uint8_t index, TrialResult_t *result);
uint8_t App_GetTrialCount(uint8_t mode);

uint8_t App_GetStatusFlags(void);
uint8_t App_GetLastVoiceCmd(void);
uint32_t App_GetLastVoiceCmdTick(void);
uint32_t App_GetPauseEnterTick(void);
uint32_t App_GetLastPausedMs(void);
uint8_t App_GetLastEvent(void);
void Calibrate_ServoRange(void);

#endif
