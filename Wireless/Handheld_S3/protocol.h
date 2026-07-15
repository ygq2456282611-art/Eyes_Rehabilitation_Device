#pragma once

#include <stdint.h>
#include <string.h>

#define EYE_FRAME_MAGIC 0x5945U
#define EYE_PROTOCOL_VERSION 4U
#define EYE_FRAME_PAYLOAD_SIZE 32U
#define EYE_CONFIG_VERSION 2U

enum EyeMessageType : uint8_t {
    EYE_MSG_PING = 1,
    EYE_MSG_PONG = 2,
    EYE_MSG_UART_TEST = 3,
    EYE_MSG_UART_ACK = 4,
    EYE_MSG_STATUS = 5,
    EYE_MSG_CONTROL = 6,
    EYE_MSG_CONTROL_ACK = 7,
    EYE_MSG_TRAINING_STATUS = 8,
    EYE_MSG_CONFIG_SET = 9,
    EYE_MSG_CONFIG_ACK = 10,
    EYE_MSG_REPORT_QUERY = 11,
    EYE_MSG_REPORT_DATA = 12,
    EYE_MSG_CONFIG_GET = 13,
    EYE_MSG_CONFIG_DATA = 14,
    EYE_MSG_SERVO_PREVIEW = 15,
    EYE_MSG_SERVO_PREVIEW_ACK = 16,
};

enum EyeControlCommand : uint8_t {
    EYE_CONTROL_START = 1,
    EYE_CONTROL_PAUSE = 2,
    EYE_CONTROL_RESUME = 3,
    EYE_CONTROL_STOP = 4,
    EYE_CONTROL_CALIBRATE = 5,
};

enum EyeControlResult : uint8_t {
    EYE_CONTROL_ACCEPTED = 0,
    EYE_CONTROL_DUPLICATE = 1,
    EYE_CONTROL_INVALID_STATE = 2,
    EYE_CONTROL_INVALID_MODE = 3,
    EYE_CONTROL_QUEUE_BUSY = 4,
    EYE_CONTROL_INVALID_FIELD = 5,
    EYE_CONTROL_OUT_OF_RANGE = 6,
    EYE_CONTROL_INVALID_ORDER = 7,
    EYE_CONTROL_VERSION_MISMATCH = 8,
};

enum EyeReportSection : uint8_t {
    EYE_REPORT_SUMMARY = 0,
    EYE_REPORT_MODE_BASIC = 1,
    EYE_REPORT_MODE_DETAIL = 2,
    EYE_REPORT_TRIAL = 3,
};

enum EyeServoAxis : uint8_t {
    EYE_SERVO_AXIS_X = 0,
    EYE_SERVO_AXIS_Y = 1,
};

enum EyeServoPreviewAction : uint8_t {
    EYE_SERVO_PREVIEW_MOVE = 0,
    EYE_SERVO_PREVIEW_PARK = 1,
    EYE_SERVO_PREVIEW_HOME = 2,
};

#pragma pack(push, 1)
struct EyeFrame {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint32_t sequence;
    uint16_t payload_length;
    uint8_t payload[EYE_FRAME_PAYLOAD_SIZE];
    uint16_t crc16;
};

struct EyeTestPayload {
    uint32_t sender_ms;
    uint32_t token;
};

struct EyeStatusPayload {
    uint8_t source;
    uint8_t reserved[3];
    uint32_t wireless_rx_count;
    uint32_t uart_rx_count;
    uint32_t invalid_count;
    uint32_t report_query_count;
    uint32_t report_uart_return_count;
    uint32_t espnow_send_success_count;
    uint32_t espnow_send_fail_count;
};

// START携带完整/自选计划；暂停、继续和停止忽略计划字段。
struct EyeControlPayload {
    uint8_t command;
    uint8_t flow_mode;
    uint8_t mode_count;
    uint8_t config_version;
    uint8_t mode_order[5];
    uint8_t reserved[3];
};

struct EyeControlAckPayload {
    uint8_t command;
    uint8_t result;
    uint8_t state;
    uint8_t mode;
    uint32_t request_sequence;
};

struct EyeTrainingConfigPayload {
    uint8_t version;
    uint8_t x_min;
    uint8_t x_home;
    uint8_t x_max;
    uint8_t y_min;
    uint8_t y_home;
    uint8_t y_max;
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
};

struct EyeConfigAckPayload {
    uint32_t request_sequence;
    uint8_t result;
    uint8_t config_version;
    uint8_t state;
    uint8_t reserved;
};

struct EyeTrainingStatusPayload {
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
    uint8_t reserved[3];
    uint8_t config_version;
};

struct EyeReportQueryPayload {
    uint32_t session_id;
    uint8_t section;
    uint8_t mode;
    uint8_t index;
    uint8_t reserved;
};

struct EyeReportDataPayload {
    uint8_t section;
    uint8_t index;
    uint8_t valid;
    uint8_t reserved;
    uint8_t data[28];
};

struct EyeServoPreviewPayload {
    uint8_t axis;
    uint8_t angle;
    uint8_t action;
    uint8_t reserved;
};

struct EyeServoPreviewAckPayload {
    uint32_t request_sequence;
    uint8_t result;
    uint8_t axis;
    uint8_t angle;
    uint8_t state;
};
#pragma pack(pop)

static_assert(sizeof(EyeFrame) == 44, "Unexpected EyeFrame packing");
static_assert(sizeof(EyeControlPayload) == 12, "Unexpected control payload packing");
static_assert(sizeof(EyeControlAckPayload) == 8, "Unexpected control ACK packing");
static_assert(sizeof(EyeTrainingConfigPayload) == 32, "Unexpected config payload packing");
static_assert(sizeof(EyeConfigAckPayload) == 8, "Unexpected config ACK packing");
static_assert(sizeof(EyeTrainingStatusPayload) == 32, "Unexpected training status packing");
static_assert(sizeof(EyeReportQueryPayload) == 8, "Unexpected report query packing");
static_assert(sizeof(EyeReportDataPayload) == 32, "Unexpected report data packing");
static_assert(sizeof(EyeStatusPayload) == 32, "Unexpected status payload packing");
static_assert(sizeof(EyeServoPreviewPayload) == 4, "Unexpected servo preview packing");
static_assert(sizeof(EyeServoPreviewAckPayload) == 8, "Unexpected servo preview ACK packing");

static inline uint16_t eye_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    while (length-- != 0U) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static inline void eye_frame_finalize(EyeFrame *frame)
{
    frame->magic = EYE_FRAME_MAGIC;
    frame->version = EYE_PROTOCOL_VERSION;
    frame->crc16 = eye_crc16((const uint8_t *)frame, sizeof(EyeFrame) - sizeof(frame->crc16));
}

static inline bool eye_frame_is_valid(const EyeFrame *frame)
{
    if ((frame->magic != EYE_FRAME_MAGIC) ||
        (frame->version != EYE_PROTOCOL_VERSION) ||
        (frame->payload_length > EYE_FRAME_PAYLOAD_SIZE)) return false;
    return frame->crc16 == eye_crc16((const uint8_t *)frame, sizeof(EyeFrame) - sizeof(frame->crc16));
}

static inline EyeFrame eye_make_test_frame(uint8_t type, uint32_t sequence, uint32_t now_ms, uint32_t token)
{
    EyeFrame frame = {};
    EyeTestPayload payload = {now_ms, token};
    frame.type = type;
    frame.sequence = sequence;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    eye_frame_finalize(&frame);
    return frame;
}

static inline EyeFrame eye_make_control_frame(uint8_t command, uint8_t flow_mode,
                                               uint8_t mode_count, const uint8_t *mode_order,
                                               uint32_t sequence)
{
    EyeFrame frame = {};
    EyeControlPayload payload = {};
    payload.command = command;
    payload.flow_mode = flow_mode;
    payload.mode_count = mode_count;
    payload.config_version = EYE_CONFIG_VERSION;
    if (mode_order != nullptr) memcpy(payload.mode_order, mode_order, sizeof(payload.mode_order));
    frame.type = EYE_MSG_CONTROL;
    frame.sequence = sequence;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    eye_frame_finalize(&frame);
    return frame;
}

static inline EyeFrame eye_make_config_frame(const EyeTrainingConfigPayload &payload, uint32_t sequence)
{
    EyeFrame frame = {};
    frame.type = EYE_MSG_CONFIG_SET;
    frame.sequence = sequence;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    eye_frame_finalize(&frame);
    return frame;
}

static inline EyeFrame eye_make_config_get_frame(uint32_t sequence)
{
    EyeFrame frame = {};
    frame.type = EYE_MSG_CONFIG_GET;
    frame.sequence = sequence;
    frame.payload_length = 0;
    eye_frame_finalize(&frame);
    return frame;
}

static inline EyeFrame eye_make_report_query(uint32_t session_id, uint8_t section,
                                              uint8_t mode, uint8_t index,
                                              uint32_t sequence)
{
    EyeFrame frame = {};
    EyeReportQueryPayload payload = {session_id, section, mode, index, 0};
    frame.type = EYE_MSG_REPORT_QUERY;
    frame.sequence = sequence;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    eye_frame_finalize(&frame);
    return frame;
}

static inline EyeFrame eye_make_servo_preview(uint8_t axis, uint8_t angle,
                                               uint8_t action, uint32_t sequence)
{
    EyeFrame frame = {};
    EyeServoPreviewPayload payload = {axis, angle, action, 0};
    frame.type = EYE_MSG_SERVO_PREVIEW;
    frame.sequence = sequence;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    eye_frame_finalize(&frame);
    return frame;
}
