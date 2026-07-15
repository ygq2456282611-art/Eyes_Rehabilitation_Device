#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <FFat.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>

#include "lvgl_v8_port.h"
#include "protocol.h"

using namespace esp_panel::board;
using namespace esp_panel::drivers;

LV_FONT_DECLARE(ui_font_20);

static constexpr uint8_t ESPNOW_CHANNEL = 1;
static constexpr uint32_t LINK_TIMEOUT_MS = 3000;
static constexpr uint32_t REQUEST_TIMEOUT_MS = 350;
static constexpr uint32_t REPORT_REQUEST_TIMEOUT_MS = 1500;
static constexpr uint8_t REQUEST_MAX_RETRIES = 3;
static constexpr uint8_t REPORT_REQUEST_MAX_RETRIES = 5;
static constexpr uint8_t STATUS_SOURCE_XIAO = 1;
static constexpr uint8_t STATUS_SOURCE_STM32 = 2;
static constexpr uint8_t APP_FLOW_FULL = 0;
static constexpr uint8_t APP_FLOW_CUSTOM = 1;
static constexpr uint8_t REPORT_HISTORY_COUNT = 20;
static constexpr uint8_t SERVO_HARD_MIN = 10;
static constexpr uint8_t SERVO_HARD_MAX = 170;
/* CFG5 accepts and migrates the previous CFG4 layout without changing payload size. */
static constexpr uint32_t CONFIG_MAGIC = 0x43464735UL;
static constexpr uint32_t LEGACY_CONFIG_MAGIC = 0x43464734UL;
static constexpr uint32_t REPORT_MAGIC = 0x52505432UL;

static constexpr uint8_t SYS_IDLE_VOICE = 0;
static constexpr uint8_t SYS_CALIBRATE = 1;
static constexpr uint8_t SYS_TRAIN = 2;
static constexpr uint8_t SYS_FEEDBACK = 3;
static constexpr uint8_t SYS_PAUSE = 4;
static constexpr uint8_t SYS_CALIB_SERVO = 5;
static constexpr uint8_t SYS_MODE_SELECT = 6;
static constexpr uint8_t SYS_TRAIN_PROMPT = 7;
static constexpr uint8_t SYS_NEXT_CONFIRM = 8;
static constexpr uint8_t SYS_MODE_ENTER_PROMPT = 9;
static constexpr uint8_t SYS_MODE_CMD_ACK_WAIT = 10;
static constexpr uint8_t SYS_SERVO_CALIB_READY = 11;

static constexpr uint8_t APP_STATUS_RUNNING = 1U << 0;
static constexpr uint8_t APP_STATUS_WAITING_PA2 = 1U << 2;
static constexpr uint8_t APP_STATUS_CONTROL_PAUSE = 1U << 4;
static constexpr uint8_t APP_STATUS_SESSION_COMPLETED = 1U << 5;

static const uint8_t XIAO_PEER_MAC[6] = {0xAC, 0x27, 0x6E, 0x7E, 0x10, 0x6C};
static const char *MODE_NAMES[5] = {
    "注视稳定", "扫视训练", "平稳追踪", "视觉聚焦", "空间忽略"
};

enum PendingKind : uint8_t {
    PENDING_NONE = 0,
    PENDING_CONTROL,
    PENDING_CONFIG,
    PENDING_CONFIG_GET,
    PENDING_REPORT,
    PENDING_SERVO_PREVIEW,
};

enum ParameterId : uint8_t {
    PARAM_X_MIN = 0, PARAM_X_HOME, PARAM_X_MAX,
    PARAM_Y_MIN, PARAM_Y_HOME, PARAM_Y_MAX,
    PARAM_SERVO_SETTLE, PARAM_FIX_DURATION,
    PARAM_SAC_TRIALS, PARAM_SAC_TIMEOUT, PARAM_SAC_INTERVAL, PARAM_SAC_PATTERN,
    PARAM_PUR_SPEED, PARAM_PUR_PATH, PARAM_PUR_ROUNDS, PARAM_PUR_TIMEOUT,
    PARAM_FOCUS_TRIALS, PARAM_FOCUS_TIMEOUT, PARAM_FOCUS_INTERVAL,
    PARAM_NEG_TRIALS, PARAM_NEG_TIMEOUT, PARAM_NEG_SIDE, PARAM_NEG_RATIO, PARAM_NEG_PATTERN,
    PARAM_HEAD_WARNING, PARAM_SAFE_TURN, PARAM_SAFE_NOD, PARAM_SAFE_TILT,
    PARAM_RESUME_TIME, PARAM_VARIATION_LIMIT,
    PARAM_COUNT
};

#pragma pack(push, 1)
struct ReportSummaryWire {
    uint32_t session_id;
    uint8_t flow_mode, mode_count, completed_modes, completed, overall_score;
    uint8_t mode_order[5];
    uint32_t total_duration_ms;
    uint8_t config_version;
    uint8_t reserved[9];
};

struct ReportModeBasicWire {
    uint32_t session_id;
    uint8_t mode, completed, score, pause_count;
    uint16_t total_trials, correct_trials, timeout_trials;
    uint16_t duration_s, avg_reaction_ms, min_reaction_ms, max_reaction_ms;
    uint16_t avg_head_offset_tenths, max_head_offset_tenths, head_variation_tenths;
};

struct ReportModeDetailWire {
    uint32_t session_id;
    uint8_t mode, valid;
    uint8_t reserved0[2];
    uint32_t head_over_limit_ms;
    uint16_t compensation_permille;
    uint16_t left_total, left_correct, right_total, right_correct;
    uint8_t trial_count, affected_side;
    int16_t asymmetry_permille;
    uint8_t reserved1[2];
};

struct ConfigBlob {
    uint32_t magic;
    EyeTrainingConfigPayload config;
    uint16_t crc;
};

struct StoredReport {
    uint8_t valid;
    ReportSummaryWire summary;
    ReportModeBasicWire basic[5];
    ReportModeDetailWire detail[5];
    uint16_t crc;
};

struct ReportStore {
    uint32_t magic;
    uint8_t version;
    uint8_t count;
    uint8_t next;
    uint8_t reserved;
    StoredReport reports[REPORT_HISTORY_COUNT];
};
#pragma pack(pop)

static_assert(sizeof(ReportSummaryWire) == 28, "Report summary size mismatch");
static_assert(sizeof(ReportModeBasicWire) == 28, "Report basic size mismatch");
static_assert(sizeof(ReportModeDetailWire) == 28, "Report detail size mismatch");

struct PendingRequest {
    EyeFrame frame;
    uint32_t sent_ms;
    uint8_t retries;
    PendingKind kind;
    bool active;
};

static QueueHandle_t receive_queue;
static QueueHandle_t report_receive_queue;
static PendingRequest pending = {};
static EyeTrainingStatusPayload training_status = {};
static EyeStatusPayload xiao_link_status = {};
static EyeStatusPayload stm32_link_status = {};
static EyeTrainingConfigPayload config = {};
static ReportStore report_store = {};
static EyeTrainingConfigPayload config_save_snapshot = {};
static ReportStore report_save_snapshot = {};
static StoredReport fetched_report = {};
static bool have_training_status;
static bool espnow_ready;
static bool config_synced;
static bool start_after_config;
static bool advanced_unlocked;
static bool config_save_pending;
static bool report_save_pending;
static bool full_redraw_pending;
static bool calibration_requested;
static bool calibration_seen_running;
static bool calibration_readback_pending;
static bool config_query_complete;
static bool config_get_for_calibration;
static bool servo_debug_active;
static bool servo_save_pending;
static bool servo_debug_snapshot_synced;
static bool initial_config_sync_pending;
static EyeTrainingConfigPayload servo_debug_snapshot = {};
static uint8_t servo_debug_axis;
static uint8_t servo_preview_angles[2] = {90, 90};
static uint8_t selected_flow = APP_FLOW_FULL;
static uint8_t custom_order[5] = {0, 1, 2, 3, 4};
static uint8_t custom_count = 1;
static uint8_t selected_parameter;
static uint8_t report_fetch_stage;
static uint8_t report_fetch_mode_pos;
static uint8_t report_history_offset;
static uint32_t report_fetch_session;
static uint32_t report_start_xiao_rx;
static uint32_t report_start_stm32_rx;
static uint32_t report_start_xiao_uart_rx;
static uint32_t report_start_screen_rx;
static uint32_t last_completed_session;
static uint32_t last_report_timeout_session;
static uint32_t last_report_timeout_ms;
static uint32_t next_sequence = 1;
static uint32_t sent_count, received_count, invalid_count;
static uint32_t last_pong_ms, last_uart_ack_ms, last_wireless_rx_ms, last_stm32_status_ms;
static uint32_t wireless_rtt_ms, full_link_rtt_ms;
static uint32_t last_report_sequence;
static uint32_t last_report_session;
static uint8_t last_report_section = 0xFFU;
static uint8_t last_report_mode = 0xFFU;

// 训练页
static lv_obj_t *wireless_value, *stm32_value, *state_value;
static lv_obj_t *flow_buttons[2], *mode_buttons[5], *mode_button_labels[5];
static lv_obj_t *start_button, *pause_button, *pause_button_label, *stop_button, *calibrate_button;
static lv_obj_t *mode_value, *progress_value, *reaction_value, *offset_value, *elapsed_value, *notice_value;
// 参数页
static lv_obj_t *parameter_dropdown, *parameter_value, *parameter_notice, *advanced_button_label;
static lv_obj_t *servo_debug_panel, *servo_axis_buttons[2], *servo_angle_label, *servo_debug_notice;
// 报告页
static lv_obj_t *report_summary_label, *report_modes_label, *report_index_label;
// 测试页
static lv_obj_t *test_wireless_value, *test_stm32_value, *test_counter_value, *test_rtt_value, *test_notice_value;
static lv_obj_t *test_report_diag_value;

static uint8_t clampU8(int value, int minimum, int maximum)
{
    if (value < minimum) return static_cast<uint8_t>(minimum);
    if (value > maximum) return static_cast<uint8_t>(maximum);
    return static_cast<uint8_t>(value);
}

static void setStandardConfig()
{
    memset(&config, 0, sizeof(config));
    config.version = EYE_CONFIG_VERSION;
    config.x_min = 80; config.x_home = 90; config.x_max = 110;
    config.y_min = 85; config.y_home = 90; config.y_max = 95;
    config.servo_settle_10ms = 10;
    config.fixation_duration_s = 15;
    config.saccade_trials = 8;
    config.saccade_timeout_100ms = 30;
    config.saccade_interval_10ms = 50;
    config.saccade_pattern = 0;
    config.pursuit_speed_deg_s = 15;
    config.pursuit_path = 0;
    config.pursuit_rounds = 1;
    config.pursuit_checkpoint_100ms = 20;
    config.focus_trials = 5;
    config.focus_timeout_s = 5;
    config.focus_interval_s = 2;
    config.focus_feedback_required = 1;
    config.neglect_trials = 6;
    config.neglect_timeout_100ms = 50;
    config.neglect_affected_side = 0;
    config.neglect_affected_ratio = 50;
    config.neglect_pattern = 0;
    config.head_warning_tenths = 50;
    config.safety_turn_deg = 20;
    config.safety_nod_deg = 20;
    config.safety_tilt_deg = 30;
    config.resume_stable_s = 3;
    config.head_variation_limit_deg = 3;
}

static uint16_t configCrc(const EyeTrainingConfigPayload &value)
{
    return eye_crc16(reinterpret_cast<const uint8_t *>(&value), sizeof(value));
}

static void saveConfigNow(const EyeTrainingConfigPayload &value)
{
    Preferences preferences;
    ConfigBlob blob = {CONFIG_MAGIC, value, configCrc(value)};
    if (preferences.begin("eye_train", false)) {
        preferences.putBytes("config", &blob, sizeof(blob));
        preferences.end();
    }
}

static void scheduleConfigSave()
{
    config_save_snapshot = config;
    config_save_pending = true;
}

static void loadConfig()
{
    setStandardConfig();
    Preferences preferences;
    ConfigBlob blob = {};
    if (preferences.begin("eye_train", true)) {
        if (preferences.getBytesLength("config") == sizeof(blob)) {
            preferences.getBytes("config", &blob, sizeof(blob));
            if ((blob.magic == CONFIG_MAGIC || blob.magic == LEGACY_CONFIG_MAGIC) &&
                blob.config.version == EYE_CONFIG_VERSION &&
                blob.crc == configCrc(blob.config)) {
                config = blob.config;
            }
        }
        preferences.end();
    }
    initial_config_sync_pending = true;
}

static uint16_t storedReportCrc(const StoredReport &report)
{
    return eye_crc16(reinterpret_cast<const uint8_t *>(&report),
                     sizeof(report) - sizeof(report.crc));
}

static void saveReportsNow(const ReportStore &store)
{
    if (!FFat.begin(true)) return;
    File file = FFat.open("/reports.bin", "w");
    if (file) {
        file.write(reinterpret_cast<const uint8_t *>(&store), sizeof(store));
        file.close();
    }
}

static void scheduleReportSave()
{
    report_save_snapshot = report_store;
    report_save_pending = true;
}

static void requestFullRedraw()
{
    full_redraw_pending = true;
}

static void serviceFullRedraw()
{
    if (!full_redraw_pending) return;
    full_redraw_pending = false;
    lv_obj_invalidate(lv_scr_act());
}

static void serviceStorageWrites()
{
    static EyeTrainingConfigPayload config_to_write;
    static ReportStore reports_to_write;
    bool write_config = false;
    bool write_reports = false;

    lvgl_port_lock(-1);
    if (config_save_pending) {
        config_to_write = config_save_snapshot;
        config_save_pending = false;
        write_config = true;
    }
    if (report_save_pending) {
        reports_to_write = report_save_snapshot;
        report_save_pending = false;
        write_reports = true;
    }
    lvgl_port_unlock();

    if (write_config) saveConfigNow(config_to_write);
    if (write_reports) saveReportsNow(reports_to_write);
}

static void loadReports()
{
    bool valid_store = false;
    if (!FFat.begin(true)) {
        Serial.println("[reports] FFat mount failed");
        return;
    }
    File file = FFat.open("/reports.bin", "r");
    if (file && file.size() == sizeof(report_store)) {
        if (file.read(reinterpret_cast<uint8_t *>(&report_store), sizeof(report_store)) ==
            sizeof(report_store) && report_store.magic == REPORT_MAGIC &&
            report_store.version == 1 && report_store.count <= REPORT_HISTORY_COUNT &&
            report_store.next < REPORT_HISTORY_COUNT) {
            valid_store = true;
            for (uint8_t i = 0; i < REPORT_HISTORY_COUNT; ++i) {
                if (report_store.reports[i].valid &&
                    report_store.reports[i].crc != storedReportCrc(report_store.reports[i])) {
                    report_store.reports[i].valid = 0;
                }
            }
        }
    }
    if (file) file.close();
    if (!valid_store) {
        memset(&report_store, 0, sizeof(report_store));
        report_store.magic = REPORT_MAGIC;
        report_store.version = 1;
    }
    Serial.printf("[reports] loaded %u record(s)\n", report_store.count);
}

static void addFetchedReport()
{
    if (!fetched_report.valid || fetched_report.summary.session_id == 0U) return;
    last_completed_session = fetched_report.summary.session_id;
    last_report_timeout_session = 0U;
    for (uint8_t i = 0; i < REPORT_HISTORY_COUNT; ++i) {
        if (report_store.reports[i].valid &&
            report_store.reports[i].summary.session_id == fetched_report.summary.session_id) {
            report_store.reports[i] = fetched_report;
            report_store.reports[i].crc = storedReportCrc(report_store.reports[i]);
            scheduleReportSave();
            return;
        }
    }
    fetched_report.crc = storedReportCrc(fetched_report);
    report_store.reports[report_store.next] = fetched_report;
    report_store.next = static_cast<uint8_t>((report_store.next + 1U) % REPORT_HISTORY_COUNT);
    if (report_store.count < REPORT_HISTORY_COUNT) ++report_store.count;
    report_history_offset = 0;
    scheduleReportSave();
}

static StoredReport *historyReport()
{
    if (report_store.count == 0U) return nullptr;
    uint8_t offset = report_history_offset;
    if (offset >= report_store.count) offset = static_cast<uint8_t>(report_store.count - 1U);
    int index = static_cast<int>(report_store.next) - 1 - offset;
    while (index < 0) index += REPORT_HISTORY_COUNT;
    return &report_store.reports[index];
}

static bool peerMacConfigured()
{
    for (uint8_t value : XIAO_PEER_MAC) if (value != 0U) return true;
    return false;
}

static bool sendFrame(const EyeFrame &frame)
{
    if (!espnow_ready) return false;
    esp_err_t result = esp_now_send(XIAO_PEER_MAC,
                                    reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
    if (result == ESP_OK) {
        ++sent_count;
        return true;
    }
    Serial.printf("ESP-NOW send failed: %d\n", static_cast<int>(result));
    return false;
}

static void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int length)
{
    if (!info || !data || length != static_cast<int>(sizeof(EyeFrame)) ||
        memcmp(info->src_addr, XIAO_PEER_MAC, sizeof(XIAO_PEER_MAC)) != 0) return;
    EyeFrame frame;
    memcpy(&frame, data, sizeof(frame));
    if (!eye_frame_is_valid(&frame)) {
        ++invalid_count;
        return;
    }
    if (frame.type == EYE_MSG_REPORT_DATA) {
        /* 报告独立排队，不能被200 ms周期状态帧挤掉。 */
        if (xQueueSend(report_receive_queue, &frame, 0) != pdTRUE) ++invalid_count;
    } else if (xQueueSend(receive_queue, &frame, 0) != pdTRUE) {
        ++invalid_count;
    }
}

static bool wirelessOnline(uint32_t now)
{
    return last_wireless_rx_ms && (now - last_wireless_rx_ms <= LINK_TIMEOUT_MS);
}

static bool stm32Online(uint32_t now)
{
    return (last_uart_ack_ms && (now - last_uart_ack_ms <= LINK_TIMEOUT_MS)) ||
           (last_stm32_status_ms && (now - last_stm32_status_ms <= LINK_TIMEOUT_MS));
}

static const char *stateText(uint8_t state)
{
    switch (state) {
        case SYS_IDLE_VOICE: return "空闲";
        case SYS_CALIBRATE: return "姿态校准";
        case SYS_TRAIN: return "训练中";
        case SYS_FEEDBACK: return "训练反馈";
        case SYS_PAUSE: return "训练暂停";
        case SYS_CALIB_SERVO: return "视野标定";
        case SYS_MODE_SELECT: return "等待选择";
        case SYS_TRAIN_PROMPT: return "准备训练";
        case SYS_NEXT_CONFIRM: return "等待下一项";
        case SYS_MODE_ENTER_PROMPT: return "模式提示";
        case SYS_MODE_CMD_ACK_WAIT: return "命令确认";
        case SYS_SERVO_CALIB_READY: return "标定准备";
        default: return "状态未知";
    }
}

static void setButtonEnabled(lv_obj_t *button, bool enabled)
{
    if (enabled) lv_obj_clear_state(button, LV_STATE_DISABLED);
    else lv_obj_add_state(button, LV_STATE_DISABLED);
}

static lv_obj_t *createButton(lv_obj_t *parent, const char *text, int x, int y, int w, int h,
                              lv_color_t color, lv_event_cb_t callback, void *user_data = nullptr)
{
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_set_size(button, w, h);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_style_radius(button, 6, 0);
    lv_obj_set_style_bg_color(button, color, 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    if (callback) lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, user_data);
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &ui_font_20, 0);
    lv_obj_center(label);
    return button;
}

static bool queueRequest(const EyeFrame &frame, PendingKind kind)
{
    if (pending.active || !wirelessOnline(millis()) || !stm32Online(millis())) return false;
    if (!sendFrame(frame)) return false;
    pending.frame = frame;
    pending.sent_ms = millis();
    pending.retries = 0;
    pending.kind = kind;
    pending.active = true;
    return true;
}

static bool sendControl(uint8_t command)
{
    uint8_t order[5] = {0, 1, 2, 3, 4};
    uint8_t count = 5;
    if (selected_flow == APP_FLOW_CUSTOM) {
        count = custom_count;
        memcpy(order, custom_order, sizeof(order));
    }
    EyeFrame frame = eye_make_control_frame(command, selected_flow, count, order, next_sequence++);
    if (!queueRequest(frame, PENDING_CONTROL)) {
        lv_label_set_text(notice_value, "设备离线或正在发送");
        return false;
    }
    lv_label_set_text(notice_value, "命令发送中");
    return true;
}

static bool sendConfig(bool start_after)
{
    EyeFrame frame = eye_make_config_frame(config, next_sequence++);
    start_after_config = start_after;
    if (!queueRequest(frame, PENDING_CONFIG)) {
        lv_label_set_text(parameter_notice, "设备离线或正在发送");
        return false;
    }
    lv_label_set_text(parameter_notice, "参数同步中");
    return true;
}

static bool sendConfigGet(bool after_calibration)
{
    EyeFrame frame = eye_make_config_get_frame(next_sequence++);
    config_get_for_calibration = after_calibration;
    if (!queueRequest(frame, PENDING_CONFIG_GET)) return false;
    return true;
}

static bool sendReportQuery(uint32_t session_id, uint8_t section, uint8_t mode, uint8_t index)
{
    EyeFrame frame = eye_make_report_query(session_id, section, mode, index, next_sequence++);
    bool queued = queueRequest(frame, PENDING_REPORT);
    Serial.printf("[report] tx seq=%lu session=%lu section=%u mode=%u queued=%u\n",
                  static_cast<unsigned long>(frame.sequence),
                  static_cast<unsigned long>(session_id), section, mode, queued ? 1U : 0U);
    return queued;
}

static bool sendServoPreview(uint8_t axis, uint8_t angle, uint8_t action)
{
    EyeFrame frame = eye_make_servo_preview(axis, angle, action, next_sequence++);
    if (!queueRequest(frame, PENDING_SERVO_PREVIEW)) return false;
    return true;
}

static int findCustomMode(uint8_t mode)
{
    for (uint8_t i = 0; i < custom_count; ++i) if (custom_order[i] == mode) return i;
    return -1;
}

static void updateModeSelection()
{
    char text[40];
    for (uint8_t mode = 0; mode < 5; ++mode) {
        int position = findCustomMode(mode);
        bool selected = selected_flow == APP_FLOW_FULL || position >= 0;
        if (selected_flow == APP_FLOW_CUSTOM && position >= 0)
            snprintf(text, sizeof(text), "%d %s", position + 1, MODE_NAMES[mode]);
        else
            snprintf(text, sizeof(text), "%s", MODE_NAMES[mode]);
        lv_label_set_text(mode_button_labels[mode], text);
        lv_obj_set_style_bg_color(mode_buttons[mode],
                                  lv_color_hex(selected ? 0x147D64 : 0xE7EDF0), 0);
        lv_obj_set_style_text_color(mode_button_labels[mode],
                                    lv_color_hex(selected ? 0xFFFFFF : 0x202A32), 0);
    }
    lv_obj_set_style_bg_color(flow_buttons[0],
                              lv_color_hex(selected_flow == APP_FLOW_FULL ? 0x176B87 : 0xD8DEE4), 0);
    lv_obj_set_style_bg_color(flow_buttons[1],
                              lv_color_hex(selected_flow == APP_FLOW_CUSTOM ? 0x176B87 : 0xD8DEE4), 0);
}

static void flowEvent(lv_event_t *event)
{
    selected_flow = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
    updateModeSelection();
}

static void modeEvent(lv_event_t *event)
{
    if (selected_flow != APP_FLOW_CUSTOM) return;
    uint8_t mode = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
    int position = findCustomMode(mode);
    if (position >= 0) {
        for (uint8_t i = static_cast<uint8_t>(position); i + 1U < custom_count; ++i)
            custom_order[i] = custom_order[i + 1U];
        if (custom_count > 0U) --custom_count;
    } else if (custom_count < 5U) {
        custom_order[custom_count++] = mode;
    }
    updateModeSelection();
}

static void startEvent(lv_event_t *)
{
    if (selected_flow == APP_FLOW_CUSTOM && custom_count == 0U) {
        lv_label_set_text(notice_value, "请至少选择一个训练项目");
        return;
    }
    if (!config_synced) {
        (void)sendConfig(true);
        return;
    }
    (void)sendControl(EYE_CONTROL_START);
}

static void pauseEvent(lv_event_t *)
{
    uint8_t command = (training_status.flags & APP_STATUS_CONTROL_PAUSE) ?
                      EYE_CONTROL_RESUME : EYE_CONTROL_PAUSE;
    (void)sendControl(command);
}

static void stopEvent(lv_event_t *) { (void)sendControl(EYE_CONTROL_STOP); }

static void calibrateEvent(lv_event_t *)
{
    (void)sendControl(EYE_CONTROL_CALIBRATE);
}

static const char *normalParameterOptions =
    "X轴最小角\nX轴初始角\nX轴最大角\nY轴最小角\nY轴初始角\nY轴最大角\n"
    "舵机稳定时间\n注视持续时间\n扫视次数\n扫视响应窗口\n扫视刺激间隔\n扫视目标模式\n"
    "追踪速度\n追踪路径\n追踪轮数\n追踪确认时间\n聚焦次数\n聚焦确认超时\n聚焦轮次间隔\n"
    "忽略训练次数\n忽略响应窗口\n忽略患侧\n患侧刺激比例\n忽略目标顺序";

static const char *advancedParameterOptions =
    "X轴最小角\nX轴初始角\nX轴最大角\nY轴最小角\nY轴初始角\nY轴最大角\n"
    "舵机稳定时间\n注视持续时间\n扫视次数\n扫视响应窗口\n扫视刺激间隔\n扫视目标模式\n"
    "追踪速度\n追踪路径\n追踪轮数\n追踪确认时间\n聚焦次数\n聚焦确认超时\n聚焦轮次间隔\n"
    "忽略训练次数\n忽略响应窗口\n忽略患侧\n患侧刺激比例\n忽略目标顺序\n"
    "头部偏移提醒\n转头保护阈值\n点头保护阈值\n侧倾保护阈值\n稳定恢复时间\n头部波动阈值";

static void updateParameterDisplay()
{
    char value[80];
    switch (selected_parameter) {
        case PARAM_X_MIN: snprintf(value, sizeof(value), "%u 度", config.x_min); break;
        case PARAM_X_HOME: snprintf(value, sizeof(value), "%u 度", config.x_home); break;
        case PARAM_X_MAX: snprintf(value, sizeof(value), "%u 度", config.x_max); break;
        case PARAM_Y_MIN: snprintf(value, sizeof(value), "%u 度", config.y_min); break;
        case PARAM_Y_HOME: snprintf(value, sizeof(value), "%u 度", config.y_home); break;
        case PARAM_Y_MAX: snprintf(value, sizeof(value), "%u 度", config.y_max); break;
        case PARAM_SERVO_SETTLE: snprintf(value, sizeof(value), "%u 毫秒", config.servo_settle_10ms * 10U); break;
        case PARAM_FIX_DURATION: snprintf(value, sizeof(value), "%u 秒", config.fixation_duration_s); break;
        case PARAM_SAC_TRIALS: snprintf(value, sizeof(value), "%u 次", config.saccade_trials); break;
        case PARAM_SAC_TIMEOUT: snprintf(value, sizeof(value), "%u 毫秒", config.saccade_timeout_100ms * 100U); break;
        case PARAM_SAC_INTERVAL: snprintf(value, sizeof(value), "%u 毫秒", config.saccade_interval_10ms * 10U); break;
        case PARAM_SAC_PATTERN: {
            const char *names[] = {"四角", "水平", "垂直", "混合"};
            snprintf(value, sizeof(value), "%s", names[config.saccade_pattern]); break;
        }
        case PARAM_PUR_SPEED: snprintf(value, sizeof(value), "%u 度每秒", config.pursuit_speed_deg_s); break;
        case PARAM_PUR_PATH: {
            const char *names[] = {"全路径", "水平", "垂直", "十字"};
            snprintf(value, sizeof(value), "%s", names[config.pursuit_path]); break;
        }
        case PARAM_PUR_ROUNDS: snprintf(value, sizeof(value), "%u 轮", config.pursuit_rounds); break;
        case PARAM_PUR_TIMEOUT: snprintf(value, sizeof(value), "%u 毫秒", config.pursuit_checkpoint_100ms * 100U); break;
        case PARAM_FOCUS_TRIALS: snprintf(value, sizeof(value), "%u 次", config.focus_trials); break;
        case PARAM_FOCUS_TIMEOUT: snprintf(value, sizeof(value), "%u 秒", config.focus_timeout_s); break;
        case PARAM_FOCUS_INTERVAL: snprintf(value, sizeof(value), "%u 秒", config.focus_interval_s); break;
        case PARAM_NEG_TRIALS: snprintf(value, sizeof(value), "%u 次", config.neglect_trials); break;
        case PARAM_NEG_TIMEOUT: snprintf(value, sizeof(value), "%u 毫秒", config.neglect_timeout_100ms * 100U); break;
        case PARAM_NEG_SIDE: snprintf(value, sizeof(value), "%s", config.neglect_affected_side ? "左侧" : "右侧"); break;
        case PARAM_NEG_RATIO: snprintf(value, sizeof(value), "%u%%", config.neglect_affected_ratio); break;
        case PARAM_NEG_PATTERN: snprintf(value, sizeof(value), "%s", config.neglect_pattern ? "患侧加权随机" : "左右交替"); break;
        case PARAM_HEAD_WARNING: snprintf(value, sizeof(value), "%u.%u 度", config.head_warning_tenths / 10U, config.head_warning_tenths % 10U); break;
        case PARAM_SAFE_TURN: snprintf(value, sizeof(value), "%u 度", config.safety_turn_deg); break;
        case PARAM_SAFE_NOD: snprintf(value, sizeof(value), "%u 度", config.safety_nod_deg); break;
        case PARAM_SAFE_TILT: snprintf(value, sizeof(value), "%u 度", config.safety_tilt_deg); break;
        case PARAM_RESUME_TIME: snprintf(value, sizeof(value), "%u 秒", config.resume_stable_s); break;
        default: snprintf(value, sizeof(value), "%u 度", config.head_variation_limit_deg); break;
    }
    lv_label_set_text(parameter_value, value);
}

static void normalizeServoRange()
{
    config.x_min = clampU8(config.x_min, SERVO_HARD_MIN, SERVO_HARD_MAX - 10);
    config.x_max = clampU8(config.x_max, SERVO_HARD_MIN + 10, SERVO_HARD_MAX);
    if (config.x_max < config.x_min + 10U) config.x_max = config.x_min + 10U;
    if (config.x_home <= config.x_min) config.x_home = config.x_min + 1U;
    if (config.x_home >= config.x_max) config.x_home = config.x_max - 1U;
    config.y_min = clampU8(config.y_min, SERVO_HARD_MIN, SERVO_HARD_MAX - 5);
    config.y_max = clampU8(config.y_max, SERVO_HARD_MIN + 5, SERVO_HARD_MAX);
    if (config.y_max < config.y_min + 5U) config.y_max = config.y_min + 5U;
    if (config.y_home <= config.y_min) config.y_home = config.y_min + 1U;
    if (config.y_home >= config.y_max) config.y_home = config.y_max - 1U;
}

static void adjustParameter(int direction)
{
    switch (selected_parameter) {
        case PARAM_X_MIN: config.x_min = clampU8(config.x_min + direction, SERVO_HARD_MIN, config.x_max - 10); break;
        case PARAM_X_HOME: config.x_home = clampU8(config.x_home + direction, config.x_min + 1, config.x_max - 1); break;
        case PARAM_X_MAX: config.x_max = clampU8(config.x_max + direction, config.x_min + 10, SERVO_HARD_MAX); break;
        case PARAM_Y_MIN: config.y_min = clampU8(config.y_min + direction, SERVO_HARD_MIN, config.y_max - 5); break;
        case PARAM_Y_HOME: config.y_home = clampU8(config.y_home + direction, config.y_min + 1, config.y_max - 1); break;
        case PARAM_Y_MAX: config.y_max = clampU8(config.y_max + direction, config.y_min + 5, SERVO_HARD_MAX); break;
        case PARAM_SERVO_SETTLE: config.servo_settle_10ms = clampU8(config.servo_settle_10ms + direction, 10, 50); break;
        case PARAM_FIX_DURATION: config.fixation_duration_s = clampU8(config.fixation_duration_s + direction, 5, 60); break;
        case PARAM_SAC_TRIALS: config.saccade_trials = clampU8(config.saccade_trials + direction, 4, 20); break;
        case PARAM_SAC_TIMEOUT: config.saccade_timeout_100ms = clampU8(config.saccade_timeout_100ms + direction, 10, 50); break;
        case PARAM_SAC_INTERVAL: config.saccade_interval_10ms = clampU8(config.saccade_interval_10ms + direction * 5, 30, 100); break;
        case PARAM_SAC_PATTERN: config.saccade_pattern = clampU8(config.saccade_pattern + direction, 0, 3); break;
        case PARAM_PUR_SPEED: config.pursuit_speed_deg_s = clampU8(config.pursuit_speed_deg_s + direction, 5, 30); break;
        case PARAM_PUR_PATH: config.pursuit_path = clampU8(config.pursuit_path + direction, 0, 3); break;
        case PARAM_PUR_ROUNDS: config.pursuit_rounds = clampU8(config.pursuit_rounds + direction, 1, 3); break;
        case PARAM_PUR_TIMEOUT: config.pursuit_checkpoint_100ms = clampU8(config.pursuit_checkpoint_100ms + direction, 10, 50); break;
        case PARAM_FOCUS_TRIALS: config.focus_trials = clampU8(config.focus_trials + direction, 2, 10); break;
        case PARAM_FOCUS_TIMEOUT: config.focus_timeout_s = clampU8(config.focus_timeout_s + direction, 2, 10); break;
        case PARAM_FOCUS_INTERVAL: config.focus_interval_s = clampU8(config.focus_interval_s + direction, 1, 10); break;
        case PARAM_NEG_TRIALS: config.neglect_trials = clampU8(config.neglect_trials + direction, 4, 20); break;
        case PARAM_NEG_TIMEOUT: config.neglect_timeout_100ms = clampU8(config.neglect_timeout_100ms + direction * 5, 20, 80); break;
        case PARAM_NEG_SIDE: config.neglect_affected_side = direction > 0 ? 1 : 0; break;
        case PARAM_NEG_RATIO: config.neglect_affected_ratio = clampU8(config.neglect_affected_ratio + direction * 10, 50, 80); break;
        case PARAM_NEG_PATTERN: config.neglect_pattern = direction > 0 ? 1 : 0; break;
        case PARAM_HEAD_WARNING: config.head_warning_tenths = clampU8(config.head_warning_tenths + direction * 5, 30, 100); break;
        case PARAM_SAFE_TURN: config.safety_turn_deg = clampU8(config.safety_turn_deg + direction, 10, 20); break;
        case PARAM_SAFE_NOD: config.safety_nod_deg = clampU8(config.safety_nod_deg + direction, 10, 20); break;
        case PARAM_SAFE_TILT: config.safety_tilt_deg = clampU8(config.safety_tilt_deg + direction, 15, 30); break;
        case PARAM_RESUME_TIME: config.resume_stable_s = clampU8(config.resume_stable_s + direction, 2, 10); break;
        case PARAM_VARIATION_LIMIT: config.head_variation_limit_deg = clampU8(config.head_variation_limit_deg + direction, 1, 3); break;
    }
    normalizeServoRange();
    config_synced = false;
    updateParameterDisplay();
    lv_label_set_text(parameter_notice, "参数已修改，点击保存并同步");
}

static void parameterDropdownEvent(lv_event_t *)
{
    selected_parameter = static_cast<uint8_t>(lv_dropdown_get_selected(parameter_dropdown));
    updateParameterDisplay();
}

static void minusEvent(lv_event_t *) { adjustParameter(-1); }
static void plusEvent(lv_event_t *) { adjustParameter(1); }

static void presetEvent(lv_event_t *event)
{
    uint8_t preset = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
    setStandardConfig();
    if (preset == 0U) {
        config.fixation_duration_s = 10;
        config.saccade_trials = 6; config.saccade_timeout_100ms = 40; config.saccade_interval_10ms = 100;
        config.pursuit_speed_deg_s = 10;
        config.focus_trials = 3;
        config.neglect_trials = 4; config.neglect_timeout_100ms = 60;
    } else if (preset == 2U) {
        config.fixation_duration_s = 30;
        config.saccade_trials = 16; config.saccade_timeout_100ms = 20; config.saccade_interval_10ms = 30;
        config.pursuit_speed_deg_s = 25; config.pursuit_rounds = 2;
        config.focus_trials = 8;
        config.neglect_trials = 12; config.neglect_timeout_100ms = 40;
        config.neglect_affected_ratio = 70; config.neglect_pattern = 1;
    }
    config_synced = false;
    updateParameterDisplay();
    lv_label_set_text(parameter_notice, preset == 0 ? "已选择初级预设" :
                      (preset == 1 ? "已选择标准预设" : "已选择进阶预设"));
}

static void syncConfigEvent(lv_event_t *)
{
    (void)sendConfig(false);
}

static void resetConfigEvent(lv_event_t *)
{
    setStandardConfig();
    config_synced = false;
    scheduleConfigSave();
    updateParameterDisplay();
    lv_label_set_text(parameter_notice, "已恢复标准参数，等待同步");
    requestFullRedraw();
}

static void advancedClickEvent(lv_event_t *)
{
    advanced_unlocked = !advanced_unlocked;
    lv_dropdown_set_options(parameter_dropdown,
                            advanced_unlocked ? advancedParameterOptions : normalParameterOptions);
    if (!advanced_unlocked && selected_parameter >= PARAM_HEAD_WARNING) {
        selected_parameter = PARAM_X_MIN;
        lv_dropdown_set_selected(parameter_dropdown, 0);
    }
    lv_label_set_text(advanced_button_label, advanced_unlocked ? "高级设置已解锁" : "点按高级 长按舵机测试");
    updateParameterDisplay();
}

static bool servoConfigValid(const EyeTrainingConfigPayload &value)
{
    return value.x_min >= SERVO_HARD_MIN && value.x_max <= SERVO_HARD_MAX &&
           value.x_min < value.x_home && value.x_home < value.x_max &&
           static_cast<uint8_t>(value.x_max - value.x_min) >= 10U &&
           value.y_min >= SERVO_HARD_MIN && value.y_max <= SERVO_HARD_MAX &&
           value.y_min < value.y_home && value.y_home < value.y_max &&
           static_cast<uint8_t>(value.y_max - value.y_min) >= 5U;
}

static void updateServoDebugDisplay()
{
    lv_label_set_text_fmt(servo_angle_label, "%s轴 当前角度 %u度",
                          servo_debug_axis == EYE_SERVO_AXIS_X ? "X" : "Y",
                          servo_preview_angles[servo_debug_axis]);
    for (uint8_t axis = 0; axis < 2U; ++axis) {
        lv_obj_set_style_bg_color(servo_axis_buttons[axis],
                                  lv_color_hex(axis == servo_debug_axis ? 0x176B87 : 0xD8DEE4), 0);
    }
}

static void servoAxisEvent(lv_event_t *event)
{
    servo_debug_axis = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
    updateServoDebugDisplay();
}

static void servoStepEvent(lv_event_t *event)
{
    if (pending.active) {
        lv_label_set_text(servo_debug_notice, "等待主控确认");
        return;
    }
    int direction = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    uint8_t angle = clampU8(static_cast<int>(servo_preview_angles[servo_debug_axis]) + direction,
                            SERVO_HARD_MIN, SERVO_HARD_MAX);
    if (sendServoPreview(servo_debug_axis, angle, EYE_SERVO_PREVIEW_MOVE)) {
        servo_preview_angles[servo_debug_axis] = angle;
        updateServoDebugDisplay();
        lv_label_set_text(servo_debug_notice, "LASER 已关闭 舵机移动中");
    }
}

static void servoSetPointEvent(lv_event_t *event)
{
    uint8_t role = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
    EyeTrainingConfigPayload candidate = config;
    uint8_t angle = servo_preview_angles[servo_debug_axis];
    uint8_t *minimum = servo_debug_axis == EYE_SERVO_AXIS_X ? &candidate.x_min : &candidate.y_min;
    uint8_t *home = servo_debug_axis == EYE_SERVO_AXIS_X ? &candidate.x_home : &candidate.y_home;
    uint8_t *maximum = servo_debug_axis == EYE_SERVO_AXIS_X ? &candidate.x_max : &candidate.y_max;
    if (role == 0U) *minimum = angle;
    else if (role == 1U) *home = angle;
    else *maximum = angle;

    if (!servoConfigValid(candidate)) {
        lv_label_set_text(servo_debug_notice, "范围无效 请保持 最小<初始<最大");
        return;
    }
    config = candidate;
    config_synced = false;
    updateParameterDisplay();
    lv_label_set_text(servo_debug_notice, "角度已保存");
}

static void closeServoDebug(bool restore_config)
{
    if (restore_config) {
        config = servo_debug_snapshot;
        config_synced = servo_debug_snapshot_synced;
        updateParameterDisplay();
    }
    servo_debug_active = false;
    lv_obj_add_flag(servo_debug_panel, LV_OBJ_FLAG_HIDDEN);
    requestFullRedraw();
}

static void servoCancelEvent(lv_event_t *)
{
    if (pending.active) {
        lv_label_set_text(servo_debug_notice, "等待主控确认");
        return;
    }
    closeServoDebug(true);
    (void)sendServoPreview(0U, 90U, EYE_SERVO_PREVIEW_PARK);
}

static void servoSaveEvent(lv_event_t *)
{
    if (!servoConfigValid(config)) {
        lv_label_set_text(servo_debug_notice, "范围无效 X跨度至少10度 Y至少5度");
        return;
    }
    servo_save_pending = true;
    if (!sendConfig(false)) {
        servo_save_pending = false;
        lv_label_set_text(servo_debug_notice, "参数发送失败");
    } else {
        lv_label_set_text(servo_debug_notice, "正在保存参数");
    }
}

static void advancedLongPressEvent(lv_event_t *)
{
    uint32_t now = millis();
    bool idle = have_training_status &&
                (training_status.state == SYS_IDLE_VOICE || training_status.state == SYS_MODE_SELECT);
    if (!wirelessOnline(now) || !stm32Online(now) || !idle || pending.active) {
        lv_label_set_text(parameter_notice, "请确认主控在线并空闲");
        return;
    }
    servo_debug_snapshot = config;
    servo_debug_snapshot_synced = config_synced;
    servo_preview_angles[0] = 90U;
    servo_preview_angles[1] = 90U;
    servo_debug_axis = EYE_SERVO_AXIS_X;
    servo_debug_active = true;
    lv_obj_clear_flag(servo_debug_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(servo_debug_panel);
    lv_label_set_text(servo_debug_notice, "移动时 LASER 关闭 范围10到170度");
    updateServoDebugDisplay();
    requestFullRedraw();
}

static void wirelessTestEvent(lv_event_t *)
{
    uint32_t now = millis();
    EyeFrame frame = eye_make_test_frame(EYE_MSG_PING, next_sequence++, now, esp_random());
    lv_label_set_text(test_notice_value, sendFrame(frame) ? "无线测试已发送" : "无线设备离线");
}

static void fullLinkTestEvent(lv_event_t *)
{
    uint32_t now = millis();
    EyeFrame frame = eye_make_test_frame(EYE_MSG_UART_TEST, next_sequence++, now, esp_random());
    lv_label_set_text(test_notice_value, sendFrame(frame) ? "链路测试已发送" : "无线设备离线");
}

static void updateReportPage()
{
    StoredReport *report = historyReport();
    char summary[180];
    char lines[700];
    if (!report) {
        lv_label_set_text(report_summary_label, "暂无训练报告");
        lv_label_set_text(report_modes_label, "训练完成后点击刷新报告");
        lv_label_set_text(report_index_label, "0 / 0");
        return;
    }
    snprintf(summary, sizeof(summary), "第 %lu 次  %s  综合表现 %u 分  有效训练 %lu 秒",
             static_cast<unsigned long>(report->summary.session_id),
             report->summary.flow_mode == APP_FLOW_FULL ? "完整训练" : "自选训练",
             report->summary.overall_score,
             static_cast<unsigned long>(report->summary.total_duration_ms / 1000U));
    lv_label_set_text(report_summary_label, summary);
    lines[0] = '\0';
    for (uint8_t i = 0; i < report->summary.mode_count && i < 5U; ++i) {
        uint8_t mode = report->summary.mode_order[i];
        const ReportModeBasicWire &basic = report->basic[mode];
        char row[150];
        uint32_t accuracy = basic.total_trials ?
                            static_cast<uint32_t>(basic.correct_trials) * 100U / basic.total_trials : 100U;
        snprintf(row, sizeof(row), "%s  %u分  正确%lu%%  反应%ums  偏移%u.%u度\n",
                 MODE_NAMES[mode], basic.score, static_cast<unsigned long>(accuracy),
                 basic.avg_reaction_ms, basic.avg_head_offset_tenths / 10U,
                 basic.avg_head_offset_tenths % 10U);
        strncat(lines, row, sizeof(lines) - strlen(lines) - 1U);
    }
    strncat(lines, "训练表现仅用于康复过程参考，不作为医疗诊断。", sizeof(lines) - strlen(lines) - 1U);
    lv_label_set_text(report_modes_label, lines);
    snprintf(summary, sizeof(summary), "%u / %u", report_history_offset + 1U, report_store.count);
    lv_label_set_text(report_index_label, summary);
}

static void previousReportEvent(lv_event_t *)
{
    if (report_history_offset + 1U < report_store.count) ++report_history_offset;
    updateReportPage();
}

static void nextReportEvent(lv_event_t *)
{
    if (report_history_offset > 0U) --report_history_offset;
    updateReportPage();
}

static void requestNextReportPart();

static void beginReportFetch(uint32_t session_id)
{
    if (pending.active) {
        Serial.printf("[report] start blocked by pending kind=%u seq=%lu\n",
                      static_cast<unsigned>(pending.kind),
                      static_cast<unsigned long>(pending.frame.sequence));
        return;
    }
    memset(&fetched_report, 0, sizeof(fetched_report));
    report_fetch_session = session_id;
    report_fetch_stage = 1U;
    report_fetch_mode_pos = 0U;
    report_start_xiao_rx = xiao_link_status.wireless_rx_count;
    report_start_stm32_rx = stm32_link_status.wireless_rx_count;
    report_start_xiao_uart_rx = xiao_link_status.uart_rx_count;
    report_start_screen_rx = received_count;
    lv_label_set_text(report_summary_label, "正在读取训练报告 0/0");
    lv_label_set_text(report_modes_label, "请保持主控在线...");
    if (!sendReportQuery(session_id, EYE_REPORT_SUMMARY, 0, 0)) {
        report_fetch_stage = 0U;
        lv_label_set_text(report_summary_label, "报告读取未开始");
    }
}

static void refreshReportEvent(lv_event_t *)
{
    if (!pending.active && report_fetch_stage != 0U && fetched_report.valid) {
        requestNextReportPart();
    } else if (!pending.active && report_fetch_stage == 1U) {
        (void)sendReportQuery(report_fetch_session, EYE_REPORT_SUMMARY, 0U, 0U);
    } else {
        beginReportFetch(0U);
    }
    if (report_fetch_stage != 0U) lv_label_set_text(report_summary_label, "正在读取训练报告...");
    requestFullRedraw();
}

static void requestNextReportPart()
{
    if (report_fetch_stage == 2U) {
        uint8_t mode = fetched_report.summary.mode_order[report_fetch_mode_pos];
        lv_label_set_text_fmt(report_summary_label, "正在读取训练报告 %u/%u",
                              static_cast<unsigned>(report_fetch_mode_pos + 1U),
                              static_cast<unsigned>(fetched_report.summary.mode_count));
        if (!sendReportQuery(report_fetch_session, EYE_REPORT_MODE_BASIC, mode, 0)) report_fetch_stage = 0U;
    } else if (report_fetch_stage == 3U) {
        uint8_t mode = fetched_report.summary.mode_order[report_fetch_mode_pos];
        lv_label_set_text_fmt(report_summary_label, "正在读取训练报告 %u/%u",
                              static_cast<unsigned>(report_fetch_mode_pos + 1U),
                              static_cast<unsigned>(fetched_report.summary.mode_count));
        if (!sendReportQuery(report_fetch_session, EYE_REPORT_MODE_DETAIL, mode, 0)) report_fetch_stage = 0U;
    }
}

static lv_obj_t *createStatusBlock(lv_obj_t *parent, const char *title, lv_obj_t **value, int x)
{
    lv_obj_t *block = lv_obj_create(parent);
    lv_obj_set_size(block, 245, 110);
    lv_obj_set_pos(block, x, 35);
    lv_obj_set_style_radius(block, 6, 0);
    lv_obj_set_style_border_width(block, 1, 0);
    lv_obj_set_style_border_color(block, lv_color_hex(0xD8DEE4), 0);
    lv_obj_set_style_bg_color(block, lv_color_hex(0xFFFFFF), 0);
    lv_obj_clear_flag(block, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *title_label = lv_label_create(block);
    lv_label_set_text(title_label, title);
    lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 4, 2);
    *value = lv_label_create(block);
    lv_label_set_text(*value, "离线");
    lv_obj_align(*value, LV_ALIGN_BOTTOM_LEFT, 4, -6);
    return block;
}

static void preparePage(lv_obj_t *tab)
{
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(tab, lv_color_hex(0xF4F6F8), 0);
    lv_obj_set_style_bg_opa(tab, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(tab, 0, 0);
    lv_obj_set_style_pad_all(tab, 0, 0);
}

static void tabChangedEvent(lv_event_t *)
{
    requestFullRedraw();
}

static void createTrainingPage(lv_obj_t *tab)
{
    preparePage(tab);
    wireless_value = lv_label_create(tab); lv_obj_set_pos(wireless_value, 18, 8);
    stm32_value = lv_label_create(tab); lv_obj_set_pos(stm32_value, 180, 8);
    state_value = lv_label_create(tab); lv_obj_set_pos(state_value, 500, 8);

    lv_obj_t *flow_label = lv_label_create(tab);
    lv_label_set_text(flow_label, "训练流程"); lv_obj_set_pos(flow_label, 18, 48);
    flow_buttons[0] = createButton(tab, "完整训练", 110, 38, 140, 46, lv_color_hex(0x176B87), flowEvent,
                                   reinterpret_cast<void *>(static_cast<uintptr_t>(APP_FLOW_FULL)));
    flow_buttons[1] = createButton(tab, "自选训练", 260, 38, 140, 46, lv_color_hex(0xD8DEE4), flowEvent,
                                   reinterpret_cast<void *>(static_cast<uintptr_t>(APP_FLOW_CUSTOM)));

    const int mode_x[5] = {18, 174, 330, 18, 174};
    const int mode_y[5] = {100, 100, 100, 158, 158};
    for (uint8_t i = 0; i < 5; ++i) {
        mode_buttons[i] = createButton(tab, MODE_NAMES[i], mode_x[i], mode_y[i], 142, 48,
                                       lv_color_hex(0x147D64), modeEvent,
                                       reinterpret_cast<void *>(static_cast<uintptr_t>(i)));
        mode_button_labels[i] = lv_obj_get_child(mode_buttons[i], 0);
    }

    start_button = createButton(tab, "开始训练", 520, 58, 240, 56,
                                lv_color_hex(0x147D64), startEvent);
    pause_button = createButton(tab, "暂停", 520, 126, 112, 52,
                                lv_color_hex(0x176B87), pauseEvent);
    pause_button_label = lv_obj_get_child(pause_button, 0);
    stop_button = createButton(tab, "停止", 648, 126, 112, 52,
                               lv_color_hex(0xB42318), stopEvent);
    calibrate_button = createButton(tab, "标定模式", 520, 188, 240, 36,
                                    lv_color_hex(0x7A4E00), calibrateEvent);

    lv_obj_t *separator = lv_obj_create(tab);
    lv_obj_set_size(separator, 760, 2); lv_obj_set_pos(separator, 18, 226);
    lv_obj_set_style_border_width(separator, 0, 0);
    lv_obj_set_style_bg_color(separator, lv_color_hex(0xD8DEE4), 0);

    mode_value = lv_label_create(tab); lv_obj_set_pos(mode_value, 18, 246);
    progress_value = lv_label_create(tab); lv_obj_set_pos(progress_value, 210, 246);
    reaction_value = lv_label_create(tab); lv_obj_set_pos(reaction_value, 485, 246);
    offset_value = lv_label_create(tab); lv_obj_set_pos(offset_value, 18, 288);
    elapsed_value = lv_label_create(tab); lv_obj_set_pos(elapsed_value, 600, 288);
    notice_value = lv_label_create(tab); lv_obj_set_pos(notice_value, 18, 340);
    lv_obj_set_style_text_color(notice_value, lv_color_hex(0x52606D), 0);
    updateModeSelection();
}

static void createParameterPage(lv_obj_t *tab)
{
    preparePage(tab);
    createButton(tab, "初级", 18, 18, 130, 46, lv_color_hex(0x176B87), presetEvent,
                 reinterpret_cast<void *>(static_cast<uintptr_t>(0)));
    createButton(tab, "标准", 160, 18, 130, 46, lv_color_hex(0x147D64), presetEvent,
                 reinterpret_cast<void *>(static_cast<uintptr_t>(1)));
    createButton(tab, "进阶", 302, 18, 130, 46, lv_color_hex(0xB56B1A), presetEvent,
                 reinterpret_cast<void *>(static_cast<uintptr_t>(2)));

    parameter_dropdown = lv_dropdown_create(tab);
    lv_obj_set_size(parameter_dropdown, 480, 52);
    lv_obj_set_pos(parameter_dropdown, 18, 94);
    lv_dropdown_set_options(parameter_dropdown, normalParameterOptions);
    lv_obj_add_event_cb(parameter_dropdown, parameterDropdownEvent, LV_EVENT_VALUE_CHANGED, nullptr);

    parameter_value = lv_label_create(tab);
    lv_obj_set_pos(parameter_value, 540, 108);
    createButton(tab, "-", 530, 158, 100, 56, lv_color_hex(0x52606D), minusEvent);
    createButton(tab, "+", 650, 158, 100, 56, lv_color_hex(0x176B87), plusEvent);

    createButton(tab, "保存并同步", 18, 250, 220, 56, lv_color_hex(0x147D64), syncConfigEvent);
    createButton(tab, "恢复默认", 252, 250, 180, 56, lv_color_hex(0x52606D), resetConfigEvent);
    lv_obj_t *advanced = createButton(tab, "点按高级 长按舵机测试", 450, 250, 300, 56,
                                      lv_color_hex(0x7A4E00), nullptr);
    advanced_button_label = lv_obj_get_child(advanced, 0);
    lv_obj_add_event_cb(advanced, advancedClickEvent, LV_EVENT_SHORT_CLICKED, nullptr);
    lv_obj_add_event_cb(advanced, advancedLongPressEvent, LV_EVENT_LONG_PRESSED, nullptr);

    parameter_notice = lv_label_create(tab);
    lv_obj_set_pos(parameter_notice, 18, 336);
    lv_label_set_text(parameter_notice, "参数修改只对下一次训练生效");
    updateParameterDisplay();
}

static void createReportPage(lv_obj_t *tab)
{
    preparePage(tab);
    report_summary_label = lv_label_create(tab);
    lv_obj_set_pos(report_summary_label, 18, 18);
    report_modes_label = lv_label_create(tab);
    lv_obj_set_width(report_modes_label, 755);
    lv_obj_set_pos(report_modes_label, 18, 70);
    createButton(tab, "<", 18, 335, 70, 48, lv_color_hex(0x52606D), previousReportEvent);
    createButton(tab, ">", 100, 335, 70, 48, lv_color_hex(0x52606D), nextReportEvent);
    createButton(tab, "刷新报告", 580, 335, 180, 48, lv_color_hex(0x176B87), refreshReportEvent);
    report_index_label = lv_label_create(tab);
    lv_obj_set_pos(report_index_label, 190, 347);
    updateReportPage();
}

static void createTestPage(lv_obj_t *tab)
{
    preparePage(tab);
    createStatusBlock(tab, "ESP-NOW / XIAO", &test_wireless_value, 18);
    createStatusBlock(tab, "STM32 / USART10", &test_stm32_value, 279);
    test_counter_value = lv_label_create(tab); lv_obj_set_pos(test_counter_value, 550, 50);
    lv_obj_set_width(test_counter_value, 120);
    test_rtt_value = lv_label_create(tab); lv_obj_set_pos(test_rtt_value, 680, 50);
    lv_obj_set_width(test_rtt_value, 100);
    createButton(tab, "无线测试", 18, 205, 245, 62, lv_color_hex(0x176B87), wirelessTestEvent);
    createButton(tab, "完整链路测试", 279, 205, 245, 62, lv_color_hex(0x176B87), fullLinkTestEvent);
    test_notice_value = lv_label_create(tab); lv_obj_set_pos(test_notice_value, 550, 224);
    lv_label_set_text(test_notice_value, "准备就绪");
    test_report_diag_value = lv_label_create(tab);
    lv_obj_set_pos(test_report_diag_value, 18, 300);
    lv_obj_set_width(test_report_diag_value, 750);
    lv_label_set_text(test_report_diag_value, "报告链路等待数据");
}

static void createServoDebugPanel(lv_obj_t *screen)
{
    servo_debug_panel = lv_obj_create(screen);
    lv_obj_set_size(servo_debug_panel, 740, 390);
    lv_obj_align(servo_debug_panel, LV_ALIGN_CENTER, 0, 12);
    lv_obj_set_style_bg_color(servo_debug_panel, lv_color_hex(0xF4F6F8), 0);
    lv_obj_set_style_bg_opa(servo_debug_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(servo_debug_panel, 2, 0);
    lv_obj_set_style_border_color(servo_debug_panel, lv_color_hex(0x176B87), 0);
    lv_obj_set_style_radius(servo_debug_panel, 6, 0);
    lv_obj_clear_flag(servo_debug_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(servo_debug_panel);
    lv_label_set_text(title, "舵机测试");
    lv_obj_set_pos(title, 20, 10);
    servo_axis_buttons[0] = createButton(servo_debug_panel, "X轴", 20, 52, 120, 50,
                                         lv_color_hex(0x176B87), servoAxisEvent,
                                         reinterpret_cast<void *>(static_cast<uintptr_t>(0)));
    servo_axis_buttons[1] = createButton(servo_debug_panel, "Y轴", 155, 52, 120, 50,
                                         lv_color_hex(0xD8DEE4), servoAxisEvent,
                                         reinterpret_cast<void *>(static_cast<uintptr_t>(1)));
    servo_angle_label = lv_label_create(servo_debug_panel);
    lv_obj_set_pos(servo_angle_label, 320, 68);
    createButton(servo_debug_panel, "-1", 20, 125, 120, 58, lv_color_hex(0x52606D),
                 servoStepEvent, reinterpret_cast<void *>(static_cast<intptr_t>(-1)));
    createButton(servo_debug_panel, "+1", 155, 125, 120, 58, lv_color_hex(0x176B87),
                 servoStepEvent, reinterpret_cast<void *>(static_cast<intptr_t>(1)));
    createButton(servo_debug_panel, "设为最小", 20, 205, 150, 52, lv_color_hex(0x52606D),
                 servoSetPointEvent, reinterpret_cast<void *>(static_cast<uintptr_t>(0)));
    createButton(servo_debug_panel, "设为初始", 185, 205, 150, 52, lv_color_hex(0x176B87),
                 servoSetPointEvent, reinterpret_cast<void *>(static_cast<uintptr_t>(1)));
    createButton(servo_debug_panel, "设为最大", 350, 205, 150, 52, lv_color_hex(0x52606D),
                 servoSetPointEvent, reinterpret_cast<void *>(static_cast<uintptr_t>(2)));
    createButton(servo_debug_panel, "保存", 520, 205, 170, 52, lv_color_hex(0x147D64), servoSaveEvent);
    createButton(servo_debug_panel, "关闭", 520, 275, 170, 52, lv_color_hex(0xB42318), servoCancelEvent);
    servo_debug_notice = lv_label_create(servo_debug_panel);
    lv_obj_set_pos(servo_debug_notice, 20, 300);
    lv_obj_set_width(servo_debug_notice, 470);
    lv_label_set_text(servo_debug_notice, "移动时 LASER 关闭");
    updateServoDebugDisplay();
    lv_obj_add_flag(servo_debug_panel, LV_OBJ_FLAG_HIDDEN);
}

static void createUi()
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0xF4F6F8), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_style_text_font(screen, &ui_font_20, 0);
    lv_obj_t *tabs = lv_tabview_create(screen, LV_DIR_TOP, 46);
    lv_obj_set_size(tabs, 800, 480);
    lv_obj_set_style_radius(tabs, 0, 0);
    lv_obj_set_style_text_font(tabs, &ui_font_20, 0);
    lv_obj_add_event_cb(tabs, tabChangedEvent, LV_EVENT_VALUE_CHANGED, nullptr);
    createTrainingPage(lv_tabview_add_tab(tabs, "训练控制"));
    createParameterPage(lv_tabview_add_tab(tabs, "参数设置"));
    createReportPage(lv_tabview_add_tab(tabs, "训练报告"));
    createTestPage(lv_tabview_add_tab(tabs, "连接测试"));
    createServoDebugPanel(screen);
}

static void handleControlAck(const EyeFrame &frame)
{
    EyeControlAckPayload payload;
    if (frame.payload_length != sizeof(payload)) { ++invalid_count; return; }
    memcpy(&payload, frame.payload, sizeof(payload));
    if (!pending.active || pending.kind != PENDING_CONTROL ||
        payload.request_sequence != pending.frame.sequence) return;
    pending.active = false;
    if ((payload.result == EYE_CONTROL_ACCEPTED || payload.result == EYE_CONTROL_DUPLICATE) &&
        payload.command == EYE_CONTROL_CALIBRATE) {
        calibration_requested = true;
        calibration_seen_running = false;
        lv_label_set_text(notice_value, "请按 PA2 开始舵机标定");
        return;
    }
    switch (payload.result) {
        case EYE_CONTROL_ACCEPTED: lv_label_set_text(notice_value, "命令已执行"); break;
        case EYE_CONTROL_DUPLICATE: lv_label_set_text(notice_value, "重复命令已确认"); break;
        case EYE_CONTROL_INVALID_STATE: lv_label_set_text(notice_value, "当前状态不可用"); break;
        case EYE_CONTROL_INVALID_ORDER: lv_label_set_text(notice_value, "自选顺序无效"); break;
        case EYE_CONTROL_VERSION_MISMATCH: lv_label_set_text(notice_value, "固件版本不匹配"); break;
        default: lv_label_set_text(notice_value, "命令执行失败"); break;
    }
}

static void handleConfigAck(const EyeFrame &frame)
{
    EyeConfigAckPayload payload;
    if (frame.payload_length != sizeof(payload)) { ++invalid_count; return; }
    memcpy(&payload, frame.payload, sizeof(payload));
    if (!pending.active || pending.kind != PENDING_CONFIG ||
        payload.request_sequence != pending.frame.sequence) return;
    pending.active = false;
    if (payload.result == EYE_CONTROL_ACCEPTED || payload.result == EYE_CONTROL_DUPLICATE) {
        config_synced = true;
        scheduleConfigSave();
        lv_label_set_text(parameter_notice, "参数已保存并同步");
        if (servo_save_pending) {
            servo_save_pending = false;
            closeServoDebug(false);
            (void)sendServoPreview(0U, 90U, EYE_SERVO_PREVIEW_HOME);
            return;
        }
        if (start_after_config) {
            start_after_config = false;
            (void)sendControl(EYE_CONTROL_START);
        }
    } else {
        if (servo_save_pending) {
            servo_save_pending = false;
            lv_label_set_text(servo_debug_notice, "保存失败 请检查范围");
        }
        start_after_config = false;
        config_synced = false;
        lv_label_set_text(parameter_notice,
                          payload.result == EYE_CONTROL_OUT_OF_RANGE ?
                          "范围超出当前标定边界" :
                          (payload.result == EYE_CONTROL_VERSION_MISMATCH ?
                           "固件版本不匹配" : "参数同步失败"));
    }
}

static void handleConfigData(const EyeFrame &frame)
{
    EyeTrainingConfigPayload received;
    if (frame.payload_length != sizeof(received)) { ++invalid_count; return; }
    if (!pending.active || pending.kind != PENDING_CONFIG_GET ||
        frame.sequence != pending.frame.sequence) return;
    memcpy(&received, frame.payload, sizeof(received));
    pending.active = false;
    config_query_complete = true;
    calibration_readback_pending = false;
    if (received.version != EYE_CONFIG_VERSION) {
        lv_label_set_text(parameter_notice, "固件配置版本不匹配");
        return;
    }

    if (config_get_for_calibration) {
        config = received;
        config_synced = true;
        initial_config_sync_pending = false;
        lv_label_set_text(notice_value, "标定完成,舵机范围已改");
    } else {
        /* 重连后由屏幕保存的机械配置恢复STM32；标定读取才反向覆盖屏幕。 */
        config_synced = false;
        initial_config_sync_pending = true;
    }
    config.focus_feedback_required = 1U;
    scheduleConfigSave();
    updateParameterDisplay();
    requestFullRedraw();
}

static void handleServoPreviewAck(const EyeFrame &frame)
{
    EyeServoPreviewAckPayload payload;
    if (frame.payload_length != sizeof(payload)) { ++invalid_count; return; }
    memcpy(&payload, frame.payload, sizeof(payload));
    if (!pending.active || pending.kind != PENDING_SERVO_PREVIEW ||
        payload.request_sequence != pending.frame.sequence) return;
    pending.active = false;
    if (payload.result == EYE_CONTROL_ACCEPTED || payload.result == EYE_CONTROL_DUPLICATE) {
        if (servo_debug_active) lv_label_set_text(servo_debug_notice, "舵机位置已确认");
    } else if (servo_debug_active) {
        lv_label_set_text(servo_debug_notice,
                          payload.result == EYE_CONTROL_INVALID_STATE ?
                          "主控不是空闲状态" : "舵机角度被主控拒绝");
    }
}

static void handleReportData(const EyeFrame &frame)
{
    EyeReportDataPayload payload;
    EyeReportQueryPayload expected;
    if (frame.payload_length != sizeof(payload)) { ++invalid_count; return; }
    memcpy(&payload, frame.payload, sizeof(payload));
    uint32_t response_session = 0U;
    uint8_t response_mode = 0U;
    memcpy(&response_session, payload.data, sizeof(response_session));
    if (payload.section == EYE_REPORT_MODE_BASIC || payload.section == EYE_REPORT_MODE_DETAIL)
        response_mode = payload.data[4];
    if (frame.sequence == last_report_sequence && payload.section == last_report_section &&
        response_session == last_report_session && response_mode == last_report_mode) {
        return;
    }
    if (!pending.active || pending.kind != PENDING_REPORT) {
        Serial.printf("[report] unexpected data seq=%lu section=%u\n",
                      static_cast<unsigned long>(frame.sequence), payload.section);
        return;
    }
    memcpy(&expected, pending.frame.payload, sizeof(expected));
    if (payload.section != expected.section) {
        Serial.printf("[report] wrong section seq=%lu got=%u expected=%u\n",
                      static_cast<unsigned long>(frame.sequence), payload.section, expected.section);
        return;
    }
    if (frame.sequence != pending.frame.sequence) {
        // The peer MAC and CRC have already been verified. Match report replies by
        // requested section/session as well, so a delayed retry is not discarded silently.
        Serial.printf("[report] sequence differs got=%lu expected=%lu; checking payload\n",
                      static_cast<unsigned long>(frame.sequence),
                      static_cast<unsigned long>(pending.frame.sequence));
    }
    last_report_sequence = frame.sequence;
    last_report_section = payload.section;
    last_report_session = response_session;
    last_report_mode = response_mode;
    pending.active = false;
    if (!payload.valid) {
        report_fetch_stage = 0;
        lv_label_set_text(report_summary_label, "报告数据暂不可用");
        return;
    }
    if (payload.section == EYE_REPORT_SUMMARY) {
        ReportSummaryWire summary;
        memcpy(&summary, payload.data, sizeof(summary));
        if (summary.session_id == 0U ||
            (expected.session_id != 0U && summary.session_id != expected.session_id)) {
            ++invalid_count;
            report_fetch_stage = 0U;
            lv_label_set_text(report_summary_label, "报告数据暂不可用");
            Serial.printf("[report] summary session mismatch got=%lu expected=%lu\n",
                          static_cast<unsigned long>(summary.session_id),
                          static_cast<unsigned long>(expected.session_id));
            return;
        }
        fetched_report.summary = summary;
        report_fetch_session = summary.session_id;
        if (fetched_report.summary.mode_count > 5U) {
            fetched_report.summary.mode_count = 5U;
        }
        fetched_report.valid = 1;
        report_fetch_stage = 2;
        report_fetch_mode_pos = 0;
        if (fetched_report.summary.mode_count == 0U) {
            report_fetch_stage = 0;
            addFetchedReport();
            updateReportPage();
            requestFullRedraw();
            return;
        }
        requestNextReportPart();
    } else if (payload.section == EYE_REPORT_MODE_BASIC) {
        ReportModeBasicWire data;
        memcpy(&data, payload.data, sizeof(data));
        if (data.session_id != report_fetch_session || data.mode != expected.mode || data.mode >= 5U) {
            ++invalid_count;
            report_fetch_stage = 0U;
            lv_label_set_text(report_summary_label, "报告数据暂不可用");
            return;
        }
        fetched_report.basic[data.mode] = data;
        report_fetch_stage = 3;
        requestNextReportPart();
    } else if (payload.section == EYE_REPORT_MODE_DETAIL) {
        ReportModeDetailWire data;
        memcpy(&data, payload.data, sizeof(data));
        if (data.session_id != report_fetch_session || data.mode != expected.mode || data.mode >= 5U) {
            ++invalid_count;
            report_fetch_stage = 0U;
            lv_label_set_text(report_summary_label, "报告数据暂不可用");
            return;
        }
        fetched_report.detail[data.mode] = data;
        ++report_fetch_mode_pos;
        if (report_fetch_mode_pos >= fetched_report.summary.mode_count) {
            report_fetch_stage = 0;
            addFetchedReport();
            updateReportPage();
            requestFullRedraw();
        } else {
            report_fetch_stage = 2;
            requestNextReportPart();
        }
    }
}

static void processReceivedFrames()
{
    EyeFrame frame;
    /* 报告帧先于状态帧处理，避免分段响应在高刷新压力下超时。 */
    while (xQueueReceive(report_receive_queue, &frame, 0) == pdTRUE) {
        ++received_count;
        last_wireless_rx_ms = millis();
        last_stm32_status_ms = millis();
        handleReportData(frame);
    }
    while (xQueueReceive(receive_queue, &frame, 0) == pdTRUE) {
        ++received_count;
        last_wireless_rx_ms = millis();
        if (frame.type == EYE_MSG_PONG && frame.payload_length == sizeof(EyeTestPayload)) {
            EyeTestPayload payload;
            memcpy(&payload, frame.payload, sizeof(payload));
            wireless_rtt_ms = millis() - payload.sender_ms;
            last_pong_ms = millis();
            lv_label_set_text(test_notice_value, "无线响应已收到");
        } else if (frame.type == EYE_MSG_UART_ACK && frame.payload_length == sizeof(EyeTestPayload)) {
            EyeTestPayload payload;
            memcpy(&payload, frame.payload, sizeof(payload));
            full_link_rtt_ms = millis() - payload.sender_ms;
            last_uart_ack_ms = millis();
            lv_label_set_text(test_notice_value, "主控响应已收到");
        } else if (frame.type == EYE_MSG_STATUS && frame.payload_length == sizeof(EyeStatusPayload)) {
            EyeStatusPayload payload;
            memcpy(&payload, frame.payload, sizeof(payload));
            if (payload.source == STATUS_SOURCE_STM32) {
                stm32_link_status = payload;
                last_stm32_status_ms = millis();
            } else if (payload.source == STATUS_SOURCE_XIAO) {
                xiao_link_status = payload;
            } else {
                ++invalid_count;
            }
        } else if (frame.type == EYE_MSG_CONTROL_ACK) {
            last_stm32_status_ms = millis();
            handleControlAck(frame);
        } else if (frame.type == EYE_MSG_CONFIG_ACK) {
            last_stm32_status_ms = millis();
            handleConfigAck(frame);
        } else if (frame.type == EYE_MSG_CONFIG_DATA) {
            last_stm32_status_ms = millis();
            handleConfigData(frame);
        } else if (frame.type == EYE_MSG_SERVO_PREVIEW_ACK) {
            last_stm32_status_ms = millis();
            handleServoPreviewAck(frame);
        } else if (frame.type == EYE_MSG_REPORT_DATA) {
            last_stm32_status_ms = millis();
            handleReportData(frame);
        } else if (frame.type == EYE_MSG_TRAINING_STATUS &&
                   frame.payload_length == sizeof(EyeTrainingStatusPayload)) {
            uint8_t previous_state = training_status.state;
            uint8_t previous_calibration_phase = training_status.calibration_phase;
            uint8_t previous_calibration_points = training_status.calibration_points;
            memcpy(&training_status, frame.payload, sizeof(training_status));
            last_stm32_status_ms = millis();
            have_training_status = true;
            if (training_status.config_version != EYE_CONFIG_VERSION)
                lv_label_set_text(notice_value, "固件版本不匹配");
            if (calibration_requested && training_status.state == SYS_CALIB_SERVO)
                calibration_seen_running = true;
            if (calibration_requested && !calibration_seen_running &&
                previous_state == SYS_SERVO_CALIB_READY &&
                training_status.state != SYS_SERVO_CALIB_READY &&
                training_status.state != SYS_CALIB_SERVO) {
                calibration_requested = false;
                lv_label_set_text(notice_value, "舵机标定已停止");
            }
            if (calibration_requested && calibration_seen_running &&
                previous_state == SYS_CALIB_SERVO && training_status.state != SYS_CALIB_SERVO) {
                calibration_requested = false;
                calibration_seen_running = false;
                if (previous_calibration_phase == 6U && previous_calibration_points == 4U) {
                    calibration_readback_pending = true;
                    config_query_complete = false;
                } else {
                    lv_label_set_text(notice_value, "舵机标定已停止");
                }
            }
            if ((training_status.flags & APP_STATUS_SESSION_COMPLETED) && !pending.active &&
                training_status.session_id != 0U &&
                training_status.session_id != last_completed_session) {
                bool timeout_cooling =
                    (training_status.session_id == last_report_timeout_session) &&
                    (millis() - last_report_timeout_ms < 5000U);
                if (!timeout_cooling) {
                    if (report_fetch_stage == 1U) {
                        (void)sendReportQuery(report_fetch_session, EYE_REPORT_SUMMARY, 0U, 0U);
                    } else if (report_fetch_stage != 0U && fetched_report.valid) {
                        requestNextReportPart();
                    } else {
                        beginReportFetch(training_status.session_id);
                    }
                }
            }
        }
    }
}

static void servicePendingRequest(uint32_t now)
{
    uint32_t timeout_ms = (pending.kind == PENDING_REPORT) ? REPORT_REQUEST_TIMEOUT_MS : REQUEST_TIMEOUT_MS;
    uint8_t max_retries = (pending.kind == PENDING_REPORT) ? REPORT_REQUEST_MAX_RETRIES : REQUEST_MAX_RETRIES;

    if (!pending.active || now - pending.sent_ms < timeout_ms) return;
    if (!wirelessOnline(now) || !stm32Online(now) || pending.retries >= max_retries) {
        PendingKind failed_kind = pending.kind;
        pending.active = false;
        if (failed_kind == PENDING_REPORT) {
            uint8_t failed_stage = report_fetch_stage;
            last_report_timeout_session = report_fetch_session;
            last_report_timeout_ms = now;
            if (report_fetch_session == 0U) {
                lv_label_set_text(report_summary_label, "报告读取超时:当前训练");
            } else {
                lv_label_set_text_fmt(report_summary_label, "报告读取超时:第%lu次训练",
                                      static_cast<unsigned long>(report_fetch_session));
            }
            lv_label_set_text_fmt(report_modes_label,
                                  "请确认连接测试页中主控在线,点刷新报告。\nS%u X+%lu M+%lu U+%lu S+%lu",
                                  static_cast<unsigned>(failed_stage),
                                  static_cast<unsigned long>(xiao_link_status.wireless_rx_count - report_start_xiao_rx),
                                  static_cast<unsigned long>(stm32_link_status.wireless_rx_count - report_start_stm32_rx),
                                  static_cast<unsigned long>(xiao_link_status.uart_rx_count - report_start_xiao_uart_rx),
                                  static_cast<unsigned long>(received_count - report_start_screen_rx));
        } else if (failed_kind == PENDING_CONFIG) {
            start_after_config = false;
            if (servo_save_pending) {
                servo_save_pending = false;
                lv_label_set_text(servo_debug_notice, "参数确认超时");
            }
            lv_label_set_text(parameter_notice, "参数确认超时");
        } else if (failed_kind == PENDING_CONFIG_GET) {
            config_query_complete = true;
            calibration_readback_pending = false;
            lv_label_set_text(parameter_notice, "读取主控参数超时");
        } else if (failed_kind == PENDING_SERVO_PREVIEW) {
            if (servo_debug_active) lv_label_set_text(servo_debug_notice, "舵机预览确认超时");
        } else {
            lv_label_set_text(notice_value, "命令确认超时");
        }
        return;
    }
    if (sendFrame(pending.frame)) {
        ++pending.retries;
        pending.sent_ms = now;
    }
}

static void updateUi()
{
    uint32_t now = millis();
    bool wireless_online = wirelessOnline(now);
    bool stm32_online = stm32Online(now);
    bool link_ready = wireless_online && stm32_online && have_training_status;
    bool idle = have_training_status &&
                (training_status.state == SYS_IDLE_VOICE || training_status.state == SYS_MODE_SELECT);
    bool manual_pause = training_status.flags & APP_STATUS_CONTROL_PAUSE;
    bool running = training_status.state == SYS_TRAIN;
    bool active = have_training_status && !idle;

    if (!link_ready) config_query_complete = false;
    if (link_ready && !config_query_complete && !pending.active) {
        bool after_calibration = calibration_readback_pending;
        if (sendConfigGet(after_calibration)) config_query_complete = true;
    }
    if (link_ready && config_query_complete && initial_config_sync_pending && !pending.active) {
        if (sendConfig(false)) initial_config_sync_pending = false;
    }

    lv_label_set_text_fmt(wireless_value, "无线 %s", wireless_online ? "在线" : "离线");
    lv_obj_set_style_text_color(wireless_value, lv_color_hex(wireless_online ? 0x147D64 : 0xB42318), 0);
    lv_label_set_text_fmt(stm32_value, "主控 %s", stm32_online ? "在线" : "离线");
    lv_obj_set_style_text_color(stm32_value, lv_color_hex(stm32_online ? 0x147D64 : 0xB42318), 0);
    lv_label_set_text_fmt(state_value, "状态 %s", have_training_status ? stateText(training_status.state) : "未知");

    uint8_t mode = training_status.mode < 5U ? training_status.mode : 0U;
    lv_label_set_text_fmt(mode_value, "模式 %s", MODE_NAMES[mode]);
    lv_label_set_text_fmt(progress_value, "进度 %u  正确 %u  超时 %u",
                          training_status.total_trials, training_status.correct_trials,
                          training_status.timeout_trials);
    lv_label_set_text_fmt(reaction_value, "反应 %u 毫秒", training_status.avg_reaction_ms);
    lv_label_set_text_fmt(offset_value, "头部偏移 %u.%u 度  波动 %u.%u 度",
                          training_status.head_offset_tenths / 10U,
                          training_status.head_offset_tenths % 10U,
                          training_status.head_variation_tenths / 10U,
                          training_status.head_variation_tenths % 10U);
    lv_label_set_text_fmt(elapsed_value, "时间 %lu 秒",
                          static_cast<unsigned long>(training_status.elapsed_ms / 1000U));

    if (!pending.active) {
        if (training_status.state == SYS_SERVO_CALIB_READY)
            lv_label_set_text(notice_value, "按 PA2 开始X/Y视野标定");
        else if (training_status.state == SYS_CALIB_SERVO) {
            const char *phase_text = "准备标定";
            switch (training_status.calibration_phase) {
                case 1: phase_text = "X轴 30→150"; break;
                case 2: phase_text = "X轴 150→30"; break;
                case 3: phase_text = "Y轴 75→135"; break;
                case 4: phase_text = "Y轴 135→75"; break;
                case 5: phase_text = "标定点"; break;
                case 6: phase_text = "保存标定值"; break;
                default: break;
            }
            lv_label_set_text_fmt(notice_value, "%s，已存 %u/4，看到边界时按 PA2",
                                  phase_text, training_status.calibration_points);
            lv_label_set_text_fmt(progress_value, "标定点 %u / 4", training_status.calibration_points);
            lv_label_set_text(mode_value, "模式 舵机视野标定");
        } else if (training_status.flags & APP_STATUS_WAITING_PA2)
            lv_label_set_text(notice_value, "等待 PA2 按键确认");
        else if (training_status.state == SYS_PAUSE && !manual_pause)
            lv_label_set_text(notice_value, "姿态异常，请保持稳定");
        else if (!link_ready)
            lv_label_set_text(notice_value, "等待设备状态");
    }

    setButtonEnabled(start_button, link_ready && idle && !pending.active &&
                     (selected_flow == APP_FLOW_FULL || custom_count > 0U));
    setButtonEnabled(pause_button, link_ready && (running || manual_pause) && !pending.active);
    setButtonEnabled(stop_button, link_ready && active && !pending.active);
    setButtonEnabled(calibrate_button, link_ready && idle && !pending.active);
    lv_label_set_text(pause_button_label, manual_pause ? "继续" : "暂停");
    for (uint8_t i = 0; i < 2U; ++i)
        setButtonEnabled(flow_buttons[i], link_ready && idle && !pending.active);
    for (uint8_t i = 0; i < 5U; ++i)
        setButtonEnabled(mode_buttons[i], link_ready && idle && !pending.active);

    lv_label_set_text(test_wireless_value, wireless_online ? "在线" : "离线");
    lv_obj_set_style_text_color(test_wireless_value, lv_color_hex(wireless_online ? 0x147D64 : 0xB42318), 0);
    lv_label_set_text(test_stm32_value, stm32_online ? "在线" : "离线");
    lv_obj_set_style_text_color(test_stm32_value, lv_color_hex(stm32_online ? 0x147D64 : 0xB42318), 0);
    lv_label_set_text_fmt(test_counter_value, "发送 %lu\n接收 %lu\n错误 %lu",
                          static_cast<unsigned long>(sent_count),
                          static_cast<unsigned long>(received_count),
                          static_cast<unsigned long>(invalid_count));
    lv_label_set_text_fmt(test_rtt_value, "无线 %lu ms\n完整 %lu ms",
                          static_cast<unsigned long>(wireless_rtt_ms),
                          static_cast<unsigned long>(full_link_rtt_ms));
    lv_label_set_text_fmt(test_report_diag_value,
                          "RPT发送 %lu  UART接收 %lu  无线已送 %lu  无线失败 %lu",
                          static_cast<unsigned long>(xiao_link_status.report_query_count),
                          static_cast<unsigned long>(xiao_link_status.report_uart_return_count),
                          static_cast<unsigned long>(xiao_link_status.espnow_send_success_count),
                          static_cast<unsigned long>(xiao_link_status.espnow_send_fail_count));
}

static bool initEspNow()
{
    if (!peerMacConfigured()) return false;
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
    Serial.print("Handheld STA MAC: ");
    Serial.println(WiFi.macAddress());
    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(onEspNowReceive);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, XIAO_PEER_MAC, sizeof(XIAO_PEER_MAC));
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    return esp_now_add_peer(&peer) == ESP_OK;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("[1/8] Handheld startup");
    next_sequence = esp_random();
    if (next_sequence == 0U) next_sequence = 1U;
    Serial.println("[2/8] Loading parameter config");
    loadConfig();

    Serial.println("[3/8] Creating receive queue");
    receive_queue = xQueueCreate(20, sizeof(EyeFrame));
    report_receive_queue = xQueueCreate(12, sizeof(EyeFrame));
    if (!receive_queue || !report_receive_queue) {
        Serial.println("FATAL: receive queue allocation failed");
        while (true) delay(1000);
    }

    Serial.println("[4/8] Initializing display board");
    Board *board = new Board();
    board->init();
#if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board->getLCD();
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcd_bus = lcd->getBus();
    if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB)
        static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
#endif
#endif
    if (!board->begin()) {
        Serial.println("FATAL: display board initialization failed");
        while (true) delay(1000);
    }
#if LVGL_PORT_AVOID_TEARING_MODE
    /* RESET后PSRAM帧缓存内容不确定，先同步清零再交给LVGL双缓冲。 */
    {
        auto active_lcd = board->getLCD();
        size_t frame_bytes = static_cast<size_t>(active_lcd->getFrameWidth()) *
                             active_lcd->getFrameHeight() * sizeof(lv_color_t);
        for (uint8_t i = 0; i < LVGL_PORT_DISP_BUFFER_NUM; ++i) {
            void *frame_buffer = active_lcd->getFrameBufferByIndex(i);
            if (frame_buffer) memset(frame_buffer, 0, frame_bytes);
        }
    }
#endif
    Serial.println("[5/8] Initializing LVGL");
    if (!lvgl_port_init(board->getLCD(), board->getTouch())) {
        Serial.println("FATAL: LVGL initialization failed");
        while (true) delay(1000);
    }

    Serial.println("[6/8] Creating user interface");
    lvgl_port_lock(-1);
    createUi();
    requestFullRedraw();
    serviceFullRedraw();
    lvgl_port_unlock();

    Serial.println("[7/8] Loading local reports");
    loadReports();
    lvgl_port_lock(-1);
    updateReportPage();
    requestFullRedraw();
    serviceFullRedraw();
    lvgl_port_unlock();

    Serial.println("[8/8] Initializing ESP-NOW");
    espnow_ready = initEspNow();
    Serial.println(espnow_ready ? "Startup complete" : "UI ready, ESP-NOW unavailable");
}

void loop()
{
    static uint32_t last_ui_update;
    uint32_t now = millis();
    lvgl_port_lock(-1);
    processReceivedFrames();
    servicePendingRequest(now);
    if (now - last_ui_update >= 100U) {
        last_ui_update = now;
        updateUi();
    }
    serviceFullRedraw();
    lvgl_port_unlock();
    serviceStorageWrites();
    delay(10);
}
