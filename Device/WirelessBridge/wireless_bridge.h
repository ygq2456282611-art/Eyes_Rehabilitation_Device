#ifndef WIRELESS_BRIDGE_H
#define WIRELESS_BRIDGE_H

#include <stdint.h>

#define WIRELESS_CONFIG_VERSION 2U

typedef struct
{
    uint32_t sequence;
    uint8_t command;
    uint8_t flow_mode;
    uint8_t mode_count;
    uint8_t config_version;
    uint8_t mode_order[5];
    uint8_t precheck_result;
} WirelessControlRequest_t;

typedef struct
{
    uint8_t version;
    uint8_t x_min, x_home, x_max;
    uint8_t y_min, y_home, y_max;
    uint8_t servo_settle_10ms;
    uint8_t fixation_duration_s;
    uint8_t saccade_trials;
    uint8_t saccade_timeout_100ms;
    uint8_t saccade_interval_10ms;
    uint8_t saccade_pattern;
    uint8_t pursuit_speed_deg_s;
    uint8_t pursuit_path;
    uint8_t pursuit_rounds;
    uint8_t pursuit_checkpoint_100ms;
    uint8_t focus_trials;
    uint8_t focus_timeout_s;
    uint8_t focus_interval_s;
    uint8_t focus_feedback_required;
    uint8_t neglect_trials;
    uint8_t neglect_timeout_100ms;
    uint8_t neglect_affected_side;
    uint8_t neglect_affected_ratio;
    uint8_t neglect_pattern;
    uint8_t head_warning_tenths;
    uint8_t safety_turn_deg;
    uint8_t safety_nod_deg;
    uint8_t safety_tilt_deg;
    uint8_t resume_stable_s;
    uint8_t head_variation_limit_deg;
} WirelessTrainingConfig_t;

typedef struct
{
    uint32_t sequence;
    WirelessTrainingConfig_t config;
    uint8_t precheck_result;
} WirelessConfigRequest_t;

typedef struct
{
    uint32_t sequence;
} WirelessConfigQuery_t;

typedef struct
{
    uint32_t sequence;
    uint32_t session_id;
    uint8_t section;
    uint8_t mode;
    uint8_t index;
} WirelessReportQuery_t;

typedef struct
{
    uint32_t sequence;
    uint8_t axis;
    uint8_t angle;
    uint8_t action;
    uint8_t precheck_result;
} WirelessServoPreviewRequest_t;

typedef struct
{
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
} WirelessTrainingStatus_t;

typedef struct
{
    uint8_t section;
    uint8_t index;
    uint8_t valid;
    uint8_t reserved;
    uint8_t data[28];
} WirelessReportData_t;

void WirelessBridge_Init(void);
void WirelessBridge_Update(uint32_t now_ms);
void WirelessBridge_OnTxComplete(void);
void WirelessBridge_OnError(void);

uint8_t WirelessBridge_TakeControl(WirelessControlRequest_t *request);
uint8_t WirelessBridge_TakeConfig(WirelessConfigRequest_t *request);
uint8_t WirelessBridge_TakeConfigQuery(WirelessConfigQuery_t *query);
uint8_t WirelessBridge_TakeReportQuery(WirelessReportQuery_t *query);
uint8_t WirelessBridge_TakeServoPreview(WirelessServoPreviewRequest_t *request);
void WirelessBridge_SendControlAck(const WirelessControlRequest_t *request,
                                   uint8_t result, uint8_t state, uint8_t mode);
void WirelessBridge_SendConfigAck(const WirelessConfigRequest_t *request,
                                  uint8_t result, uint8_t state);
void WirelessBridge_SendConfigData(const WirelessConfigQuery_t *query,
                                   const WirelessTrainingConfig_t *config);
void WirelessBridge_SendTrainingStatus(const WirelessTrainingStatus_t *status);
void WirelessBridge_SendReportData(const WirelessReportQuery_t *query,
                                   const WirelessReportData_t *data);
void WirelessBridge_SendServoPreviewAck(const WirelessServoPreviewRequest_t *request,
                                        uint8_t result, uint8_t state);

uint8_t WirelessBridge_IsOnline(void);
uint32_t WirelessBridge_GetValidRxCount(void);
uint32_t WirelessBridge_GetInvalidRxCount(void);
uint32_t WirelessBridge_GetTxCount(void);
uint32_t WirelessBridge_GetLastSequence(void);
uint32_t WirelessBridge_GetLastCommandSequence(void);
uint8_t WirelessBridge_GetLastCommandResult(void);

#endif
