#include "wireless_bridge.h"

#include "usart.h"

#include <string.h>

#define EYE_FRAME_MAGIC 0x5945U
#define EYE_PROTOCOL_VERSION 4U
#define EYE_FRAME_PAYLOAD_SIZE 32U
#define EYE_UART_RX_BUFFER_SIZE 132U
#define EYE_LINK_TIMEOUT_MS 3000U
#define EYE_CONTROL_QUEUE_SIZE 4U
#define EYE_TX_QUEUE_SIZE 12U
#define EYE_UART_TX_TIMEOUT_MS 100U

#define EYE_CONTROL_ACCEPTED 0U
#define EYE_CONTROL_DUPLICATE 1U
#define EYE_CONTROL_QUEUE_BUSY 4U
#define EYE_CONTROL_VERSION_MISMATCH 8U

typedef enum
{
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
    EYE_MSG_SERVO_PREVIEW_ACK = 16
} EyeMessageType_t;

#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint32_t sequence;
    uint16_t payload_length;
    uint8_t payload[EYE_FRAME_PAYLOAD_SIZE];
    uint16_t crc16;
} EyeFrame_t;

typedef struct
{
    uint8_t source;
    uint8_t reserved[3];
    uint32_t wireless_rx_count;
    uint32_t uart_rx_count;
    uint32_t invalid_count;
    uint32_t report_query_count;
    uint32_t report_uart_return_count;
    uint32_t espnow_send_success_count;
    uint32_t espnow_send_fail_count;
} EyeStatusPayload_t;

/* 三端保持相同字段顺序和紧凑布局，编译期尺寸检查用于阻止协议漂移。 */
typedef struct
{
    uint8_t command;
    uint8_t flow_mode;
    uint8_t mode_count;
    uint8_t config_version;
    uint8_t mode_order[5];
    uint8_t reserved[3];
} EyeControlPayload_t;

typedef struct
{
    uint8_t command;
    uint8_t result;
    uint8_t state;
    uint8_t mode;
    uint32_t request_sequence;
} EyeControlAckPayload_t;

typedef struct
{
    uint32_t request_sequence;
    uint8_t result;
    uint8_t config_version;
    uint8_t state;
    uint8_t reserved;
} EyeConfigAckPayload_t;

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
    uint8_t reserved[3];
    uint8_t config_version;
} EyeTrainingStatusPayload_t;

typedef struct
{
    uint32_t session_id;
    uint8_t section;
    uint8_t mode;
    uint8_t index;
    uint8_t reserved;
} EyeReportQueryPayload_t;

typedef struct
{
    uint8_t axis;
    uint8_t angle;
    uint8_t action;
    uint8_t reserved;
} EyeServoPreviewPayload_t;

typedef struct
{
    uint32_t request_sequence;
    uint8_t result;
    uint8_t axis;
    uint8_t angle;
    uint8_t state;
} EyeServoPreviewAckPayload_t;
#pragma pack(pop)

typedef char EyeFrameSizeCheck[(sizeof(EyeFrame_t) == 44U) ? 1 : -1];
typedef char EyeControlPayloadSizeCheck[(sizeof(EyeControlPayload_t) == 12U) ? 1 : -1];
typedef char EyeControlAckSizeCheck[(sizeof(EyeControlAckPayload_t) == 8U) ? 1 : -1];
typedef char EyeConfigSizeCheck[(sizeof(WirelessTrainingConfig_t) == 32U) ? 1 : -1];
typedef char EyeConfigAckSizeCheck[(sizeof(EyeConfigAckPayload_t) == 8U) ? 1 : -1];
typedef char EyeTrainingStatusSizeCheck[(sizeof(EyeTrainingStatusPayload_t) == 32U) ? 1 : -1];
typedef char EyeReportQuerySizeCheck[(sizeof(EyeReportQueryPayload_t) == 8U) ? 1 : -1];
typedef char EyeReportDataSizeCheck[(sizeof(WirelessReportData_t) == 32U) ? 1 : -1];
typedef char EyeStatusPayloadSizeCheck[(sizeof(EyeStatusPayload_t) == 32U) ? 1 : -1];
typedef char EyeServoPreviewSizeCheck[(sizeof(EyeServoPreviewPayload_t) == 4U) ? 1 : -1];
typedef char EyeServoPreviewAckSizeCheck[(sizeof(EyeServoPreviewAckPayload_t) == 8U) ? 1 : -1];

static uint8_t rx_dma_buffer[EYE_UART_RX_BUFFER_SIZE];
static uint8_t stream_buffer[EYE_UART_RX_BUFFER_SIZE];
static uint16_t stream_length;
static EyeFrame_t tx_queue[EYE_TX_QUEUE_SIZE];
static EyeFrame_t active_tx_frame;
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint8_t tx_queue_used;
static volatile uint8_t tx_busy;
static volatile uint32_t tx_start_ms;
static volatile uint32_t valid_rx_count;
static volatile uint32_t invalid_rx_count;
static volatile uint32_t tx_count;
static volatile uint32_t tx_fail_count;
static volatile uint32_t last_sequence;
static volatile uint32_t last_valid_rx_ms;
static WirelessControlRequest_t control_queue[EYE_CONTROL_QUEUE_SIZE];
static WirelessControlRequest_t overflow_request;
static volatile uint8_t control_head;
static volatile uint8_t control_tail;
static volatile uint8_t overflow_pending;
static volatile uint32_t last_command_sequence;
static volatile uint8_t last_command_result;
static WirelessConfigRequest_t pending_config_request;
static WirelessConfigQuery_t pending_config_query;
static WirelessReportQuery_t pending_report_query;
static WirelessServoPreviewRequest_t pending_servo_preview;
static volatile uint8_t config_pending;
static volatile uint8_t config_query_pending;
static volatile uint8_t report_query_pending;
static volatile uint8_t servo_preview_pending;
static volatile uint32_t last_config_sequence;
static volatile uint32_t last_servo_preview_sequence;

static uint16_t WirelessBridge_Crc16(const uint8_t *data, uint32_t length);
static uint8_t WirelessBridge_FrameIsValid(const EyeFrame_t *frame);
static void WirelessBridge_FinalizeFrame(EyeFrame_t *frame);
static void WirelessBridge_StartReceive(void);
static void WirelessBridge_ParseBytes(const uint8_t *data, uint16_t size);
static void WirelessBridge_HandleFrame(const EyeFrame_t *frame);
static void WirelessBridge_QueueAck(const EyeFrame_t *request);
static void WirelessBridge_QueueStatus(const EyeFrame_t *request);
static void WirelessBridge_QueueControl(const EyeFrame_t *frame);
static void WirelessBridge_QueueConfig(const EyeFrame_t *frame);
static void WirelessBridge_QueueConfigQuery(const EyeFrame_t *frame);
static void WirelessBridge_QueueReportQuery(const EyeFrame_t *frame);
static void WirelessBridge_QueueServoPreview(const EyeFrame_t *frame);
static uint8_t WirelessBridge_QueueTxFrame(const EyeFrame_t *frame, uint8_t replace_status);

void WirelessBridge_Init(void)
{
    memset(rx_dma_buffer, 0, sizeof(rx_dma_buffer));
    memset(stream_buffer, 0, sizeof(stream_buffer));
    memset(tx_queue, 0, sizeof(tx_queue));
    memset(&active_tx_frame, 0, sizeof(active_tx_frame));
    tx_head = 0U;
    tx_tail = 0U;
    tx_queue_used = 0U;
    tx_busy = 0U;
    tx_start_ms = 0U;
    valid_rx_count = 0U;
    invalid_rx_count = 0U;
    tx_count = 0U;
    tx_fail_count = 0U;
    last_sequence = 0U;
    last_valid_rx_ms = 0U;
    control_head = 0U;
    control_tail = 0U;
    overflow_pending = 0U;
    config_pending = 0U;
    config_query_pending = 0U;
    report_query_pending = 0U;
    servo_preview_pending = 0U;
    last_command_sequence = 0U;
    last_command_result = EYE_CONTROL_ACCEPTED;
    last_config_sequence = 0U;
    last_servo_preview_sequence = 0U;
    memset(&pending_config_request, 0, sizeof(pending_config_request));
    memset(&pending_config_query, 0, sizeof(pending_config_query));
    memset(&pending_report_query, 0, sizeof(pending_report_query));
    memset(&pending_servo_preview, 0, sizeof(pending_servo_preview));
    stream_length = 0U;
    WirelessBridge_StartReceive();
}

void WirelessBridge_Update(uint32_t now_ms)
{
    uint8_t start_tx = 0U;

    /* 正常发送一帧约 4 ms。超过 100 ms 仍未完成说明完成中断或 UART 状态异常，
       主循环主动解锁；当前帧可由屏幕请求重试，后续状态帧不能被永久堵住。 */
    if (tx_busy != 0U && (now_ms - tx_start_ms) > EYE_UART_TX_TIMEOUT_MS)
    {
        (void)HAL_UART_AbortTransmit(&huart10);
        __disable_irq();
        tx_busy = 0U;
        tx_start_ms = 0U;
        tx_fail_count++;
        __enable_irq();
    }

    __disable_irq();
    if ((tx_queue_used != 0U) && (tx_busy == 0U))
    {
        memcpy(&active_tx_frame, &tx_queue[tx_tail], sizeof(active_tx_frame));
        tx_tail = (uint8_t)((tx_tail + 1U) % EYE_TX_QUEUE_SIZE);
        tx_queue_used--;
        tx_busy = 1U;
        start_tx = 1U;
    }
    __enable_irq();

    if (start_tx != 0U)
    {
        if (HAL_UART_Transmit_IT(&huart10, (uint8_t *)&active_tx_frame, sizeof(active_tx_frame)) != HAL_OK)
        {
            tx_busy = 0U;
            tx_start_ms = 0U;
            tx_fail_count++;
        }
        else
        {
            tx_start_ms = now_ms;
        }
    }
}

void WirelessBridge_OnTxComplete(void)
{
    tx_busy = 0U;
    tx_start_ms = 0U;
    tx_count++;
}

void WirelessBridge_OnError(void)
{
    tx_busy = 0U;
    tx_start_ms = 0U;
    (void)HAL_UART_AbortTransmit(&huart10);
    (void)HAL_UART_AbortReceive(&huart10);
    WirelessBridge_StartReceive();
}

uint8_t WirelessBridge_IsOnline(void)
{
    if (valid_rx_count == 0U)
    {
        return 0U;
    }
    return (uint8_t)((HAL_GetTick() - last_valid_rx_ms) <= EYE_LINK_TIMEOUT_MS);
}

uint32_t WirelessBridge_GetValidRxCount(void)
{
    return valid_rx_count;
}

uint32_t WirelessBridge_GetInvalidRxCount(void)
{
    return invalid_rx_count + tx_fail_count;
}

uint32_t WirelessBridge_GetTxCount(void)
{
    return tx_count;
}

uint32_t WirelessBridge_GetLastSequence(void)
{
    return last_sequence;
}

uint32_t WirelessBridge_GetLastCommandSequence(void)
{
    return last_command_sequence;
}

uint8_t WirelessBridge_GetLastCommandResult(void)
{
    return last_command_result;
}

uint8_t WirelessBridge_TakeControl(WirelessControlRequest_t *request)
{
    uint8_t available = 0U;

    if (request == 0)
        return 0U;

    __disable_irq();
    if (control_tail != control_head)
    {
        *request = control_queue[control_tail];
        control_tail = (uint8_t)((control_tail + 1U) % EYE_CONTROL_QUEUE_SIZE);
        available = 1U;
    }
    else if (overflow_pending != 0U)
    {
        *request = overflow_request;
        overflow_pending = 0U;
        available = 1U;
    }
    __enable_irq();

    if (available != 0U && request->precheck_result == EYE_CONTROL_ACCEPTED &&
        request->sequence == last_command_sequence)
    {
        request->precheck_result = EYE_CONTROL_DUPLICATE;
    }
    return available;
}

uint8_t WirelessBridge_TakeConfig(WirelessConfigRequest_t *request)
{
    if (request == 0) return 0U;
    __disable_irq();
    if (config_pending == 0U)
    {
        __enable_irq();
        return 0U;
    }
    *request = pending_config_request;
    config_pending = 0U;
    __enable_irq();
    if (request->sequence == last_config_sequence)
        request->precheck_result = EYE_CONTROL_DUPLICATE;
    return 1U;
}

uint8_t WirelessBridge_TakeConfigQuery(WirelessConfigQuery_t *query)
{
    if (query == 0) return 0U;
    __disable_irq();
    if (config_query_pending == 0U)
    {
        __enable_irq();
        return 0U;
    }
    *query = pending_config_query;
    config_query_pending = 0U;
    __enable_irq();
    return 1U;
}

uint8_t WirelessBridge_TakeReportQuery(WirelessReportQuery_t *query)
{
    if (query == 0) return 0U;
    __disable_irq();
    if (report_query_pending == 0U)
    {
        __enable_irq();
        return 0U;
    }
    *query = pending_report_query;
    report_query_pending = 0U;
    __enable_irq();
    return 1U;
}

uint8_t WirelessBridge_TakeServoPreview(WirelessServoPreviewRequest_t *request)
{
    if (request == 0) return 0U;
    __disable_irq();
    if (servo_preview_pending == 0U)
    {
        __enable_irq();
        return 0U;
    }
    *request = pending_servo_preview;
    servo_preview_pending = 0U;
    __enable_irq();
    if (request->sequence == last_servo_preview_sequence)
        request->precheck_result = EYE_CONTROL_DUPLICATE;
    return 1U;
}

void WirelessBridge_SendControlAck(const WirelessControlRequest_t *request,
                                   uint8_t result, uint8_t state, uint8_t mode)
{
    EyeControlAckPayload_t payload;
    EyeFrame_t frame;

    if (request == 0)
        return;

    memset(&frame, 0, sizeof(frame));
    payload.command = request->command;
    payload.result = result;
    payload.state = state;
    payload.mode = mode;
    payload.request_sequence = request->sequence;
    frame.type = EYE_MSG_CONTROL_ACK;
    frame.sequence = request->sequence;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    WirelessBridge_FinalizeFrame(&frame);

    /* 先记录已处理序号，即使本次ACK发送受阻，重发也不会再次执行命令。 */
    if (result != EYE_CONTROL_QUEUE_BUSY)
    {
        last_command_sequence = request->sequence;
        last_command_result = result;
    }
    if (WirelessBridge_QueueTxFrame(&frame, 1U) == 0U)
        tx_fail_count++;
}

void WirelessBridge_SendConfigAck(const WirelessConfigRequest_t *request,
                                  uint8_t result, uint8_t state)
{
    EyeConfigAckPayload_t payload;
    EyeFrame_t frame;
    if (request == 0) return;
    memset(&frame, 0, sizeof(frame));
    payload.request_sequence = request->sequence;
    payload.result = result;
    payload.config_version = WIRELESS_CONFIG_VERSION;
    payload.state = state;
    payload.reserved = 0U;
    frame.type = EYE_MSG_CONFIG_ACK;
    frame.sequence = request->sequence;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    WirelessBridge_FinalizeFrame(&frame);
    if (result != EYE_CONTROL_QUEUE_BUSY)
    {
        last_config_sequence = request->sequence;
    }
    if (WirelessBridge_QueueTxFrame(&frame, 1U) == 0U) tx_fail_count++;
}

void WirelessBridge_SendConfigData(const WirelessConfigQuery_t *query,
                                   const WirelessTrainingConfig_t *config)
{
    EyeFrame_t frame;
    if (query == 0 || config == 0) return;
    memset(&frame, 0, sizeof(frame));
    frame.type = EYE_MSG_CONFIG_DATA;
    frame.sequence = query->sequence;
    frame.payload_length = sizeof(*config);
    memcpy(frame.payload, config, sizeof(*config));
    WirelessBridge_FinalizeFrame(&frame);
    if (WirelessBridge_QueueTxFrame(&frame, 1U) == 0U) tx_fail_count++;
}

void WirelessBridge_SendTrainingStatus(const WirelessTrainingStatus_t *status)
{
    EyeTrainingStatusPayload_t payload;
    EyeFrame_t frame;

    if (status == 0)
        return;

    memset(&frame, 0, sizeof(frame));
    memset(&payload, 0, sizeof(payload));
    payload.state = status->state;
    payload.mode = status->mode;
    payload.flow_mode = status->flow_mode;
    payload.flags = status->flags;
    payload.mode_index = status->mode_index;
    payload.mode_count = status->mode_count;
    payload.total_trials = status->total_trials;
    payload.correct_trials = status->correct_trials;
    payload.timeout_trials = status->timeout_trials;
    payload.elapsed_ms = status->elapsed_ms;
    payload.avg_reaction_ms = status->avg_reaction_ms;
    payload.head_offset_tenths = status->head_offset_tenths;
    payload.head_variation_tenths = status->head_variation_tenths;
    payload.session_id = status->session_id;
    payload.calibration_phase = status->calibration_phase;
    payload.calibration_points = status->calibration_points;
    payload.config_version = WIRELESS_CONFIG_VERSION;
    frame.type = EYE_MSG_TRAINING_STATUS;
    frame.sequence = last_command_sequence;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    WirelessBridge_FinalizeFrame(&frame);
    (void)WirelessBridge_QueueTxFrame(&frame, 0U);
}

void WirelessBridge_SendReportData(const WirelessReportQuery_t *query,
                                   const WirelessReportData_t *data)
{
    EyeFrame_t frame;
    if (query == 0 || data == 0) return;
    memset(&frame, 0, sizeof(frame));
    frame.type = EYE_MSG_REPORT_DATA;
    frame.sequence = query->sequence;
    frame.payload_length = sizeof(*data);
    memcpy(frame.payload, data, sizeof(*data));
    WirelessBridge_FinalizeFrame(&frame);
    if (WirelessBridge_QueueTxFrame(&frame, 1U) == 0U) tx_fail_count++;
}

void WirelessBridge_SendServoPreviewAck(const WirelessServoPreviewRequest_t *request,
                                        uint8_t result, uint8_t state)
{
    EyeServoPreviewAckPayload_t payload;
    EyeFrame_t frame;
    if (request == 0) return;
    memset(&frame, 0, sizeof(frame));
    payload.request_sequence = request->sequence;
    payload.result = result;
    payload.axis = request->axis;
    payload.angle = request->angle;
    payload.state = state;
    frame.type = EYE_MSG_SERVO_PREVIEW_ACK;
    frame.sequence = request->sequence;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    WirelessBridge_FinalizeFrame(&frame);
    if (result != EYE_CONTROL_QUEUE_BUSY)
        last_servo_preview_sequence = request->sequence;
    if (WirelessBridge_QueueTxFrame(&frame, 1U) == 0U) tx_fail_count++;
}

static uint8_t WirelessBridge_QueueTxFrame(const EyeFrame_t *frame, uint8_t replace_status)
{
    uint32_t primask;
    uint8_t queued = 0U;
    uint8_t i;
    uint8_t index;

    if (frame == 0)
        return 0U;

    /* Priority frames may replace queued periodic status frames when the UART is busy. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (tx_queue_used < EYE_TX_QUEUE_SIZE)
    {
        memcpy(&tx_queue[tx_head], frame, sizeof(tx_queue[tx_head]));
        tx_head = (uint8_t)((tx_head + 1U) % EYE_TX_QUEUE_SIZE);
        tx_queue_used++;
        queued = 1U;
    }
    else if (replace_status != 0U)
    {
        for (i = 0U; i < tx_queue_used; i++)
        {
            index = (uint8_t)((tx_tail + i) % EYE_TX_QUEUE_SIZE);
            if (tx_queue[index].type == EYE_MSG_STATUS ||
                tx_queue[index].type == EYE_MSG_TRAINING_STATUS)
            {
                memcpy(&tx_queue[index], frame, sizeof(tx_queue[index]));
                queued = 1U;
                break;
            }
        }
    }
    else if (frame->type == EYE_MSG_STATUS || frame->type == EYE_MSG_TRAINING_STATUS)
    {
        queued = 1U;
    }
    if (primask == 0U)
        __enable_irq();
    return queued;
}

static uint16_t WirelessBridge_Crc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t bit;

    while (length-- != 0U)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint8_t WirelessBridge_FrameIsValid(const EyeFrame_t *frame)
{
    if ((frame->magic != EYE_FRAME_MAGIC) ||
        (frame->version != EYE_PROTOCOL_VERSION) ||
        (frame->payload_length > EYE_FRAME_PAYLOAD_SIZE))
    {
        return 0U;
    }

    return (uint8_t)(frame->crc16 ==
        WirelessBridge_Crc16((const uint8_t *)frame, sizeof(EyeFrame_t) - sizeof(frame->crc16)));
}

static void WirelessBridge_FinalizeFrame(EyeFrame_t *frame)
{
    frame->magic = EYE_FRAME_MAGIC;
    frame->version = EYE_PROTOCOL_VERSION;
    frame->crc16 = WirelessBridge_Crc16((const uint8_t *)frame,
                                        sizeof(EyeFrame_t) - sizeof(frame->crc16));
}

static void WirelessBridge_StartReceive(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_dma_buffer, sizeof(rx_dma_buffer)) == HAL_OK)
    {
        if (huart10.hdmarx != 0)
        {
            __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
        }
    }
}

static void WirelessBridge_ParseBytes(const uint8_t *data, uint16_t size)
{
    uint16_t offset = 0U;
    EyeFrame_t frame;

    if (size > (uint16_t)(sizeof(stream_buffer) - stream_length))
    {
        stream_length = 0U;
        invalid_rx_count++;
    }
    if (size > sizeof(stream_buffer))
    {
        data = &data[size - sizeof(stream_buffer)];
        size = sizeof(stream_buffer);
    }
    memcpy(&stream_buffer[stream_length], data, size);
    stream_length = (uint16_t)(stream_length + size);

    while ((uint16_t)(stream_length - offset) >= 2U)
    {
        if ((stream_buffer[offset] != (uint8_t)(EYE_FRAME_MAGIC & 0xFFU)) ||
            (stream_buffer[offset + 1U] != (uint8_t)(EYE_FRAME_MAGIC >> 8)))
        {
            offset++;
            invalid_rx_count++;
            continue;
        }

        if ((uint16_t)(stream_length - offset) < sizeof(EyeFrame_t))
        {
            break;
        }

        memcpy(&frame, &stream_buffer[offset], sizeof(frame));
        if (WirelessBridge_FrameIsValid(&frame) == 0U)
        {
            offset++;
            invalid_rx_count++;
            continue;
        }

        offset = (uint16_t)(offset + sizeof(EyeFrame_t));
        WirelessBridge_HandleFrame(&frame);
    }

    if (offset != 0U)
    {
        memmove(stream_buffer, &stream_buffer[offset], stream_length - offset);
        stream_length = (uint16_t)(stream_length - offset);
    }
}

static void WirelessBridge_HandleFrame(const EyeFrame_t *frame)
{
    valid_rx_count++;
    last_sequence = frame->sequence;
    last_valid_rx_ms = HAL_GetTick();

    if (frame->type == EYE_MSG_UART_TEST)
    {
        WirelessBridge_QueueAck(frame);
    }
    else if (frame->type == EYE_MSG_STATUS)
    {
        WirelessBridge_QueueStatus(frame);
    }
    else if (frame->type == EYE_MSG_CONTROL && frame->payload_length == sizeof(EyeControlPayload_t))
    {
        WirelessBridge_QueueControl(frame);
    }
    else if (frame->type == EYE_MSG_CONFIG_SET &&
             frame->payload_length == sizeof(WirelessTrainingConfig_t))
    {
        WirelessBridge_QueueConfig(frame);
    }
    else if (frame->type == EYE_MSG_CONFIG_GET && frame->payload_length == 0U)
    {
        WirelessBridge_QueueConfigQuery(frame);
    }
    else if (frame->type == EYE_MSG_REPORT_QUERY &&
             frame->payload_length == sizeof(EyeReportQueryPayload_t))
    {
        WirelessBridge_QueueReportQuery(frame);
    }
    else if (frame->type == EYE_MSG_SERVO_PREVIEW &&
             frame->payload_length == sizeof(EyeServoPreviewPayload_t))
    {
        WirelessBridge_QueueServoPreview(frame);
    }
}

static void WirelessBridge_QueueControl(const EyeFrame_t *frame)
{
    EyeControlPayload_t payload;
    WirelessControlRequest_t request;
    uint8_t next_head;

    memcpy(&payload, frame->payload, sizeof(payload));
    request.sequence = frame->sequence;
    request.command = payload.command;
    request.flow_mode = payload.flow_mode;
    request.mode_count = payload.mode_count;
    request.config_version = payload.config_version;
    memcpy(request.mode_order, payload.mode_order, sizeof(request.mode_order));
    request.precheck_result = EYE_CONTROL_ACCEPTED;

    /* ISR producer/main-loop consumer ring. One overflow slot reports QUEUE_BUSY. */
    next_head = (uint8_t)((control_head + 1U) % EYE_CONTROL_QUEUE_SIZE);
    if (next_head == control_tail)
    {
        if (overflow_pending == 0U)
        {
            request.precheck_result = EYE_CONTROL_QUEUE_BUSY;
            overflow_request = request;
            overflow_pending = 1U;
        }
        else
        {
            invalid_rx_count++;
        }
        return;
    }

    control_queue[control_head] = request;
    __DMB();
    control_head = next_head;
}

static void WirelessBridge_QueueConfig(const EyeFrame_t *frame)
{
    WirelessConfigRequest_t request;
    memset(&request, 0, sizeof(request));
    request.sequence = frame->sequence;
    memcpy(&request.config, frame->payload, sizeof(request.config));
    request.precheck_result = EYE_CONTROL_ACCEPTED;
    if (request.config.version != WIRELESS_CONFIG_VERSION)
        request.precheck_result = EYE_CONTROL_VERSION_MISMATCH;
    if (config_pending != 0U)
    {
        invalid_rx_count++;
        return;
    }
    pending_config_request = request;
    __DMB();
    config_pending = 1U;
}

static void WirelessBridge_QueueConfigQuery(const EyeFrame_t *frame)
{
    if (config_query_pending != 0U)
    {
        invalid_rx_count++;
        return;
    }
    pending_config_query.sequence = frame->sequence;
    __DMB();
    config_query_pending = 1U;
}

static void WirelessBridge_QueueReportQuery(const EyeFrame_t *frame)
{
    EyeReportQueryPayload_t payload;
    if (report_query_pending != 0U)
    {
        invalid_rx_count++;
        return;
    }
    memcpy(&payload, frame->payload, sizeof(payload));
    pending_report_query.sequence = frame->sequence;
    pending_report_query.session_id = payload.session_id;
    pending_report_query.section = payload.section;
    pending_report_query.mode = payload.mode;
    pending_report_query.index = payload.index;
    __DMB();
    report_query_pending = 1U;
}

static void WirelessBridge_QueueServoPreview(const EyeFrame_t *frame)
{
    EyeServoPreviewPayload_t payload;
    WirelessServoPreviewRequest_t request;
    memcpy(&payload, frame->payload, sizeof(payload));
    request.sequence = frame->sequence;
    request.axis = payload.axis;
    request.angle = payload.angle;
    request.action = payload.action;
    request.precheck_result = EYE_CONTROL_ACCEPTED;
    if (servo_preview_pending != 0U)
    {
        invalid_rx_count++;
        return;
    }
    pending_servo_preview = request;
    __DMB();
    servo_preview_pending = 1U;
}

static void WirelessBridge_QueueAck(const EyeFrame_t *request)
{
    EyeFrame_t frame;

    memcpy(&frame, request, sizeof(frame));
    frame.type = EYE_MSG_UART_ACK;
    WirelessBridge_FinalizeFrame(&frame);
    if (WirelessBridge_QueueTxFrame(&frame, 1U) == 0U)
        tx_fail_count++;
}

static void WirelessBridge_QueueStatus(const EyeFrame_t *request)
{
    EyeStatusPayload_t payload;
    EyeFrame_t frame;

    memset(&frame, 0, sizeof(frame));
    memset(&payload, 0, sizeof(payload));
    payload.source = 2U;
    payload.wireless_rx_count = valid_rx_count;
    payload.uart_rx_count = tx_count;
    payload.invalid_count = invalid_rx_count + tx_fail_count;

    frame.type = EYE_MSG_STATUS;
    frame.sequence = request->sequence;
    frame.payload_length = sizeof(payload);
    memcpy(frame.payload, &payload, sizeof(payload));
    WirelessBridge_FinalizeFrame(&frame);
    (void)WirelessBridge_QueueTxFrame(&frame, 0U);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART10)
    {
        WirelessBridge_ParseBytes(rx_dma_buffer, Size);
        WirelessBridge_StartReceive();
    }
}
