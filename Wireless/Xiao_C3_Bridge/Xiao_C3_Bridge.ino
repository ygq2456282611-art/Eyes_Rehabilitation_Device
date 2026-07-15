#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "protocol.h"

static constexpr uint8_t ESPNOW_CHANNEL = 1;
static constexpr uint32_t STATUS_PERIOD_MS = 1000;
static constexpr uint8_t STATUS_SOURCE_XIAO = 1;
static constexpr int UART_RX_PIN = 20;  // XIAO D7
static constexpr int UART_TX_PIN = 21;  // XIAO D6
static constexpr uint8_t WIRELESS_HIGH_QUEUE_SIZE = 24;
static constexpr uint8_t WIRELESS_STATUS_QUEUE_SIZE = 4;
static constexpr uint32_t WIRELESS_SEND_TIMEOUT_MS = 120;
static constexpr uint8_t WIRELESS_PRIORITY_MAX_RETRIES = 3;

// Replace this with the STA MAC printed by MacAddress.ino on the handheld S3.
static const uint8_t HANDHELD_PEER_MAC[6] = {0x44, 0x1B, 0xF6, 0x97, 0xF6, 0xB8};

static HardwareSerial DeviceSerial(1);
static QueueHandle_t wireless_queue;
static QueueHandle_t wireless_high_queue;
static QueueHandle_t wireless_status_queue;
static EyeFrame wireless_active_frame;
static uint8_t uart_buffer[sizeof(EyeFrame) * 3U];
static size_t uart_length;
static volatile bool wireless_send_busy;
static volatile bool wireless_retry_pending;
static volatile uint8_t wireless_active_retries;
static uint32_t wireless_send_start_ms;
static uint32_t wireless_rx_count;
static uint32_t wireless_tx_drop_count;
static uint32_t uart_rx_count;
static uint32_t invalid_count;
static uint32_t report_query_count;
static uint32_t report_data_count;
static uint32_t espnow_send_success_count;
static uint32_t espnow_send_fail_count;
static uint32_t status_sequence = 1;
static bool espnow_ready;
static bool last_report_query_valid;
static bool cached_report_valid;
static EyeFrame last_report_query;
static EyeFrame cached_report_frame;

static bool peerMacConfigured()
{
    for (uint8_t value : HANDHELD_PEER_MAC) {
        if (value != 0U) {
            return true;
        }
    }
    return false;
}

static bool isPriorityFrame(uint8_t type)
{
    return type != EYE_MSG_STATUS && type != EYE_MSG_TRAINING_STATUS;
}

static bool queueWireless(const EyeFrame &frame)
{
    if (!espnow_ready) {
        return false;
    }

    QueueHandle_t queue = isPriorityFrame(frame.type) ? wireless_high_queue : wireless_status_queue;
    if (xQueueSend(queue, &frame, 0) == pdTRUE) return true;

    if (!isPriorityFrame(frame.type)) {
        EyeFrame discarded;
        (void)xQueueReceive(wireless_status_queue, &discarded, 0);
        if (xQueueSend(wireless_status_queue, &frame, 0) == pdTRUE) return true;
    }

    ++wireless_tx_drop_count;
    return false;
}

static void serviceWirelessTx()
{
    if (!espnow_ready) {
        return;
    }

    if (wireless_send_busy && (millis() - wireless_send_start_ms < WIRELESS_SEND_TIMEOUT_MS)) {
        return;
    }

    if (wireless_send_busy) {
        wireless_send_busy = false;
        ++espnow_send_fail_count;
        if (isPriorityFrame(wireless_active_frame.type) &&
            wireless_active_retries < WIRELESS_PRIORITY_MAX_RETRIES) {
            ++wireless_active_retries;
            wireless_retry_pending = true;
        } else {
            ++wireless_tx_drop_count;
        }
    }

    if (!wireless_retry_pending) {
        if (xQueueReceive(wireless_high_queue, &wireless_active_frame, 0) != pdTRUE &&
            xQueueReceive(wireless_status_queue, &wireless_active_frame, 0) != pdTRUE) return;
        wireless_active_retries = 0U;
    } else {
        wireless_retry_pending = false;
    }

    /* 先标记 busy 再调用 ESP-NOW，避免发送回调抢先执行后又被主循环写回 busy。 */
    wireless_send_busy = true;
    wireless_send_start_ms = millis();
    esp_err_t result = esp_now_send(HANDHELD_PEER_MAC,
                                    reinterpret_cast<const uint8_t *>(&wireless_active_frame),
                                    sizeof(wireless_active_frame));
    if (result != ESP_OK) {
        wireless_send_busy = false;
        ++espnow_send_fail_count;
        if (isPriorityFrame(wireless_active_frame.type) &&
            wireless_active_retries < WIRELESS_PRIORITY_MAX_RETRIES) {
            ++wireless_active_retries;
            wireless_retry_pending = true;
        } else {
            ++wireless_tx_drop_count;
        }
    }
}

static void onEspNowSent(const uint8_t *, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        ++espnow_send_success_count;
    } else {
        ++espnow_send_fail_count;
        if (isPriorityFrame(wireless_active_frame.type) &&
            wireless_active_retries < WIRELESS_PRIORITY_MAX_RETRIES) {
            ++wireless_active_retries;
            wireless_retry_pending = true;
        } else {
            ++wireless_tx_drop_count;
        }
    }
    wireless_send_busy = false;
}

static void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int length)
{
    if ((info == nullptr) || (data == nullptr) || (length != static_cast<int>(sizeof(EyeFrame))) ||
        (memcmp(info->src_addr, HANDHELD_PEER_MAC, sizeof(HANDHELD_PEER_MAC)) != 0)) {
        return;
    }

    EyeFrame frame;
    memcpy(&frame, data, sizeof(frame));
    if (!eye_frame_is_valid(&frame)) {
        ++invalid_count;
        return;
    }

    if (xQueueSend(wireless_queue, &frame, 0) != pdTRUE) ++invalid_count;
}

static bool sameReportQuery(const EyeFrame &left, const EyeFrame &right)
{
    return left.sequence == right.sequence &&
           left.payload_length == sizeof(EyeReportQueryPayload) &&
           right.payload_length == sizeof(EyeReportQueryPayload) &&
           memcmp(left.payload, right.payload, sizeof(EyeReportQueryPayload)) == 0;
}

static void processWirelessFrames()
{
    EyeFrame frame;
    while (xQueueReceive(wireless_queue, &frame, 0) == pdTRUE) {
        ++wireless_rx_count;
        if (frame.type == EYE_MSG_PING) {
            frame.type = EYE_MSG_PONG;
            eye_frame_finalize(&frame);
            (void)queueWireless(frame);
        } else if (frame.type == EYE_MSG_REPORT_QUERY) {
            ++report_query_count;
            EyeReportQueryPayload query;
            memcpy(&query, frame.payload, sizeof(query));

            if (last_report_query_valid && sameReportQuery(frame, last_report_query)) {
                // 屏幕重试同一分段时优先重发缓存，避免重复占用 USART10。
                if (cached_report_valid && cached_report_frame.sequence == frame.sequence) {
                    (void)queueWireless(cached_report_frame);
                    Serial.printf("[report] cache replay seq=%lu section=%u\n",
                                  static_cast<unsigned long>(frame.sequence), query.section);
                }
                continue;
            }

            last_report_query = frame;
            last_report_query_valid = true;
            cached_report_valid = false;
            size_t written = DeviceSerial.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
            DeviceSerial.flush();
            Serial.printf("[report] query #%lu seq=%lu session=%lu section=%u mode=%u bytes=%u\n",
                          static_cast<unsigned long>(report_query_count),
                          static_cast<unsigned long>(frame.sequence),
                          static_cast<unsigned long>(query.session_id), query.section, query.mode,
                          static_cast<unsigned>(written));
            if (written != sizeof(frame)) ++invalid_count;
        } else if ((frame.type == EYE_MSG_UART_TEST) || (frame.type == EYE_MSG_CONTROL) ||
                   (frame.type == EYE_MSG_CONFIG_SET) || (frame.type == EYE_MSG_CONFIG_GET) ||
                   (frame.type == EYE_MSG_SERVO_PREVIEW)) {
            // 配置、控制和报告查询保持原帧转发，由STM32统一校验序号与CRC。
            size_t written = DeviceSerial.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
            DeviceSerial.flush();
            if (written != sizeof(frame)) {
                ++invalid_count;
                Serial.printf("[uart] short write type=%u bytes=%u\n",
                              frame.type, static_cast<unsigned>(written));
            }
        }
    }
}

static void consumeUartBytes(size_t count)
{
    if (count >= uart_length) {
        uart_length = 0U;
        return;
    }
    memmove(uart_buffer, uart_buffer + count, uart_length - count);
    uart_length -= count;
}

static void handleUartFrame(const EyeFrame &frame)
{
    ++uart_rx_count;
    if ((frame.type == EYE_MSG_UART_ACK) || (frame.type == EYE_MSG_STATUS) ||
        (frame.type == EYE_MSG_CONTROL_ACK) || (frame.type == EYE_MSG_TRAINING_STATUS) ||
        (frame.type == EYE_MSG_CONFIG_ACK) || (frame.type == EYE_MSG_CONFIG_DATA) ||
        (frame.type == EYE_MSG_REPORT_DATA) || (frame.type == EYE_MSG_SERVO_PREVIEW_ACK)) {
        bool queued = queueWireless(frame);
        if (frame.type == EYE_MSG_REPORT_DATA) {
            ++report_data_count;
            if (last_report_query_valid && frame.sequence == last_report_query.sequence) {
                cached_report_frame = frame;
                cached_report_valid = true;
            }
            EyeReportDataPayload payload;
            memcpy(&payload, frame.payload, sizeof(payload));
            Serial.printf("[report] data #%lu seq=%lu section=%u valid=%u queued=%u\n",
                          static_cast<unsigned long>(report_data_count),
                          static_cast<unsigned long>(frame.sequence), payload.section,
                          payload.valid, queued ? 1U : 0U);
        }
    }
}

static void processUart()
{
    while ((DeviceSerial.available() > 0) && (uart_length < sizeof(uart_buffer))) {
        uart_buffer[uart_length++] = static_cast<uint8_t>(DeviceSerial.read());
    }

    const uint8_t magic_low = static_cast<uint8_t>(EYE_FRAME_MAGIC & 0xFFU);
    const uint8_t magic_high = static_cast<uint8_t>(EYE_FRAME_MAGIC >> 8);
    while (uart_length >= 2U) {
        if ((uart_buffer[0] != magic_low) || (uart_buffer[1] != magic_high)) {
            consumeUartBytes(1U);
            ++invalid_count;
            continue;
        }
        if (uart_length < sizeof(EyeFrame)) {
            break;
        }

        EyeFrame frame;
        memcpy(&frame, uart_buffer, sizeof(frame));
        if (!eye_frame_is_valid(&frame)) {
            consumeUartBytes(1U);
            ++invalid_count;
            continue;
        }

        consumeUartBytes(sizeof(frame));
        handleUartFrame(frame);
    }
}

static void sendStatus()
{
    EyeFrame frame = {};
    EyeStatusPayload payload = {
        STATUS_SOURCE_XIAO, {0, 0, 0}, wireless_rx_count, uart_rx_count,
        invalid_count + wireless_tx_drop_count,
        report_query_count, report_data_count,
        espnow_send_success_count, espnow_send_fail_count
    };

    frame.type = EYE_MSG_STATUS;
    frame.sequence = status_sequence++;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    eye_frame_finalize(&frame);

    (void)queueWireless(frame);
    DeviceSerial.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
}

static bool initEspNow()
{
    if (!peerMacConfigured()) {
        Serial.println("Set HANDHELD_PEER_MAC before pairing.");
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    Serial.print("XIAO STA MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed.");
        return false;
    }

    esp_now_register_recv_cb(onEspNowReceive);
    esp_now_register_send_cb(onEspNowSent);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, HANDHELD_PEER_MAC, sizeof(HANDHELD_PEER_MAC));
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("ESP-NOW peer registration failed.");
        return false;
    }

    return true;
}

void setup()
{
    Serial.begin(115200);
    DeviceSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    delay(500);

    wireless_queue = xQueueCreate(12, sizeof(EyeFrame));
    wireless_high_queue = xQueueCreate(WIRELESS_HIGH_QUEUE_SIZE, sizeof(EyeFrame));
    wireless_status_queue = xQueueCreate(WIRELESS_STATUS_QUEUE_SIZE, sizeof(EyeFrame));
    assert(wireless_queue != nullptr && wireless_high_queue != nullptr &&
           wireless_status_queue != nullptr);
    wireless_send_busy = false;
    wireless_retry_pending = false;
    wireless_active_retries = 0U;
    espnow_ready = initEspNow();
}

void loop()
{
    static uint32_t last_status_ms;

    processWirelessFrames();
    processUart();
    serviceWirelessTx();
    if ((millis() - last_status_ms) >= STATUS_PERIOD_MS) {
        last_status_ms = millis();
        sendStatus();
    }
    serviceWirelessTx();
    delay(2);
}
