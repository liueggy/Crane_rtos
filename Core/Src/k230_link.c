#include "k230_link.h"

#include "app_state.h"
#include "cmsis_os2.h"

#include <string.h>

#define K230_PROTOCOL_HEADER_0        0xAAU
#define K230_PROTOCOL_HEADER_1        0x55U
#define K230_PROTOCOL_VERSION         0x01U
#define K230_CMD_REQUEST_ONCE         0x10U
#define K230_CMD_STREAM_START         0x11U
#define K230_CMD_STREAM_STOP          0x12U
#define K230_CMD_SET_TASK             0x20U
#define K230_CMD_RESULT_RESPONSE      0x90U
#define K230_CMD_TASK_ACK             0xA0U
#define K230_TARGET_WIRE_SIZE         6U
#define K230_MAX_PAYLOAD_SIZE         (2U + K230_MAX_DETECTIONS * K230_TARGET_WIRE_SIZE)
#define K230_MAX_FRAME_SIZE           (10U + K230_MAX_PAYLOAD_SIZE)
#define K230_RX_DMA_SIZE              128U
#define K230_RX_RING_SIZE             512U
#define K230_TX_BUFFER_SIZE           192U
#define K230_EVENT_RX_DATA            (1UL << 0)
#define K230_AUTO_REQUEST_PERIOD_MS   200U
#define K230_REQUEST_TIMEOUT_MS       1000U
#define K230_ONLINE_TIMEOUT_MS        1200U
#define K230_TASK_SWITCH_TIMEOUT_MS   10000U

typedef struct
{
  uint8_t bytes[K230_MAX_FRAME_SIZE];
  uint16_t length;
  uint16_t expected_length;
} K230FrameParser;

static UART_HandleTypeDef *g_uart;
static osEventFlagsId_t g_events;
static osMutexId_t g_tx_mutex;
static uint8_t g_rx_dma[K230_RX_DMA_SIZE];
static uint8_t g_rx_ring[K230_RX_RING_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint32_t g_rx_overflows;
static uint8_t g_tx_buffer[K230_TX_BUFFER_SIZE];
static volatile uint8_t g_tx_busy;
static uint16_t g_next_sequence;
static uint16_t g_pending_sequence;
static uint32_t g_request_deadline;
static uint32_t g_last_request_tick;
static uint32_t g_last_valid_tick;
static uint8_t g_request_pending;
static uint8_t g_stream_active;
static volatile uint8_t g_task_switch_queued;
static volatile uint8_t g_task_switch_pending;
static volatile K230VisionTask g_selected_task;
static volatile K230VisionTask g_requested_task;
static volatile K230TaskSwitchState g_task_switch_state;
static uint16_t g_task_switch_sequence;
static uint32_t g_task_switch_deadline;
static K230FrameParser g_parser;
static K230VisionResult g_result;
static volatile uint32_t g_result_generation;
static K230LinkStats g_stats;

static uint16_t ReadU16Le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void WriteU16Le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

static uint16_t Crc16Modbus(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t index = 0U; index < length; ++index)
  {
    crc ^= data[index];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
    }
  }
  return crc;
}

static void StartReceive(void)
{
  if ((g_uart == NULL) || (g_uart->hdmarx == NULL)) return;
  __HAL_DMA_DISABLE_IT(g_uart->hdmarx, DMA_IT_HT);
  (void)HAL_UARTEx_ReceiveToIdle_DMA(g_uart, g_rx_dma, sizeof(g_rx_dma));
}

static void RingPush(const uint8_t *data, uint16_t size)
{
  for (uint16_t index = 0U; index < size; ++index)
  {
    uint16_t next = (uint16_t)((g_rx_head + 1U) % K230_RX_RING_SIZE);
    if (next == g_rx_tail)
    {
      ++g_rx_overflows;
      break;
    }
    g_rx_ring[g_rx_head] = data[index];
    g_rx_head = next;
  }
}

static uint8_t RingPop(uint8_t *value)
{
  if ((value == NULL) || (g_rx_tail == g_rx_head)) return 0U;
  *value = g_rx_ring[g_rx_tail];
  g_rx_tail = (uint16_t)((g_rx_tail + 1U) % K230_RX_RING_SIZE);
  return 1U;
}

static void PublishResult(const K230VisionResult *result)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  ++g_result_generation;
  __DMB();
  g_result = *result;
  __DMB();
  ++g_result_generation;
  __set_PRIMASK(primask);
}

static void ParserReset(void)
{
  g_parser.length = 0U;
  g_parser.expected_length = 0U;
}

static void HandleResultFrame(const uint8_t *frame, uint16_t frame_length)
{
  K230VisionResult result = {0};
  uint16_t payload_length = ReadU16Le(&frame[6]);
  const uint8_t *payload = &frame[8];
  uint8_t wire_count;

  if ((frame_length != (uint16_t)(payload_length + 10U)) ||
      (frame[2] != K230_PROTOCOL_VERSION) ||
      (frame[3] != K230_CMD_RESULT_RESPONSE) ||
      (payload_length < 2U))
  {
    ++g_stats.format_errors;
    return;
  }

  wire_count = payload[1];
  if (((payload[0] != K230_TASK_NUMBER) && (payload[0] != K230_TASK_BEAN)) ||
      (payload_length != (uint16_t)(2U + (uint16_t)wire_count * K230_TARGET_WIRE_SIZE)))
  {
    ++g_stats.format_errors;
    return;
  }

  /* 模型切换期间可能收到切换前已在途的结果，不能发布为新任务数据。 */
  if (g_task_switch_queued || g_task_switch_pending) return;

  result.valid = 1U;
  result.task = payload[0];
  result.sequence = ReadU16Le(&frame[4]);
  result.received_tick_ms = HAL_GetTick();
  result.truncated = (wire_count > K230_MAX_DETECTIONS) ? 1U : 0U;
  result.count = (wire_count > K230_MAX_DETECTIONS) ? K230_MAX_DETECTIONS : wire_count;

  for (uint8_t index = 0U; index < result.count; ++index)
  {
    const uint8_t *item = &payload[2U + (uint16_t)index * K230_TARGET_WIRE_SIZE];
    result.targets[index].semantic = item[0];
    result.targets[index].confidence_percent = (item[1] > 100U) ? 100U : item[1];
    result.targets[index].center_x = ReadU16Le(&item[2]);
    result.targets[index].center_y = ReadU16Le(&item[4]);
  }

  PublishResult(&result);
  g_selected_task = (K230VisionTask)result.task;
  g_requested_task = (K230VisionTask)result.task;
  g_task_switch_state = K230_TASK_SWITCH_IDLE;
  ++g_stats.valid_frames;
  g_last_valid_tick = result.received_tick_ms;
  AppState_SetK230Online(1U);
  if (g_request_pending && (result.sequence == g_pending_sequence)) g_request_pending = 0U;
}

static void HandleTaskAckFrame(const uint8_t *frame, uint16_t frame_length)
{
  uint16_t payload_length = ReadU16Le(&frame[6]);
  const uint8_t *payload = &frame[8];
  uint16_t sequence = ReadU16Le(&frame[4]);

  if ((frame_length != 12U) || (payload_length != 2U) ||
      ((payload[0] != K230_TASK_NUMBER) && (payload[0] != K230_TASK_BEAN)))
  {
    ++g_stats.format_errors;
    return;
  }
  if (!g_task_switch_pending || (sequence != g_task_switch_sequence)) return;

  g_task_switch_pending = 0U;
  if ((payload[1] == 0U) && (payload[0] == (uint8_t)g_requested_task))
  {
    g_selected_task = (K230VisionTask)payload[0];
    g_task_switch_state = K230_TASK_SWITCH_IDLE;
    g_last_valid_tick = HAL_GetTick();
    g_last_request_tick = 0U;
    AppState_SetK230Online(1U);
  }
  else
  {
    g_task_switch_state = K230_TASK_SWITCH_FAILED;
  }
}

static void ValidateFrame(void)
{
  uint16_t received_crc = ReadU16Le(&g_parser.bytes[g_parser.length - 2U]);
  uint16_t calculated_crc = Crc16Modbus(g_parser.bytes, (uint16_t)(g_parser.length - 2U));
  if (received_crc != calculated_crc)
  {
    ++g_stats.crc_errors;
    return;
  }
  if (g_parser.bytes[2] != K230_PROTOCOL_VERSION)
  {
    ++g_stats.format_errors;
    return;
  }
  if (g_parser.bytes[3] == K230_CMD_RESULT_RESPONSE)
    HandleResultFrame(g_parser.bytes, g_parser.length);
  else if (g_parser.bytes[3] == K230_CMD_TASK_ACK)
    HandleTaskAckFrame(g_parser.bytes, g_parser.length);
  else
    ++g_stats.format_errors;
}

static void ParserFeed(uint8_t value)
{
  if (g_parser.length == 0U)
  {
    if (value == K230_PROTOCOL_HEADER_0) g_parser.bytes[g_parser.length++] = value;
    return;
  }
  if (g_parser.length == 1U)
  {
    if (value == K230_PROTOCOL_HEADER_1)
      g_parser.bytes[g_parser.length++] = value;
    else if (value != K230_PROTOCOL_HEADER_0)
      ParserReset();
    return;
  }

  if (g_parser.length >= sizeof(g_parser.bytes))
  {
    ++g_stats.format_errors;
    ParserReset();
    if (value == K230_PROTOCOL_HEADER_0) g_parser.bytes[g_parser.length++] = value;
    return;
  }
  g_parser.bytes[g_parser.length++] = value;

  if (g_parser.length == 8U)
  {
    uint16_t payload_length = ReadU16Le(&g_parser.bytes[6]);
    if (payload_length > K230_MAX_PAYLOAD_SIZE)
    {
      ++g_stats.format_errors;
      ParserReset();
      return;
    }
    g_parser.expected_length = (uint16_t)(payload_length + 10U);
  }
  if ((g_parser.expected_length != 0U) && (g_parser.length == g_parser.expected_length))
  {
    ValidateFrame();
    ParserReset();
  }
}

static uint8_t SendCommand(uint8_t command, const uint8_t *payload,
                           uint16_t payload_length, uint16_t *sequence_out)
{
  uint16_t sequence;
  uint16_t crc;
  uint8_t started = 0U;

  uint16_t frame_length = (uint16_t)(payload_length + 10U);
  if ((g_uart == NULL) || (g_tx_mutex == NULL) ||
      (frame_length > sizeof(g_tx_buffer)) ||
      ((payload_length != 0U) && (payload == NULL))) return 0U;
  if (osMutexAcquire(g_tx_mutex, 0U) != osOK) return 0U;
  if (!g_tx_busy)
  {
    sequence = g_next_sequence++;
    g_tx_buffer[0] = K230_PROTOCOL_HEADER_0;
    g_tx_buffer[1] = K230_PROTOCOL_HEADER_1;
    g_tx_buffer[2] = K230_PROTOCOL_VERSION;
    g_tx_buffer[3] = command;
    WriteU16Le(&g_tx_buffer[4], sequence);
    WriteU16Le(&g_tx_buffer[6], payload_length);
    if (payload_length != 0U) memcpy(&g_tx_buffer[8], payload, payload_length);
    crc = Crc16Modbus(g_tx_buffer, (uint16_t)(8U + payload_length));
    WriteU16Le(&g_tx_buffer[8U + payload_length], crc);
    g_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(g_uart, g_tx_buffer, frame_length) == HAL_OK)
    {
      started = 1U;
      if (sequence_out != NULL) *sequence_out = sequence;
    }
    else
    {
      g_tx_busy = 0U;
    }
  }
  (void)osMutexRelease(g_tx_mutex);
  return started;
}

void K230Link_Init(UART_HandleTypeDef *uart)
{
  g_uart = uart;
  g_events = osEventFlagsNew(NULL);
  g_tx_mutex = osMutexNew(NULL);
  g_rx_head = 0U;
  g_rx_tail = 0U;
  g_rx_overflows = 0U;
  g_tx_busy = 0U;
  g_next_sequence = 1U;
  g_request_pending = 0U;
  g_stream_active = 0U;
  g_task_switch_queued = 0U;
  g_task_switch_pending = 0U;
  g_selected_task = K230_TASK_NUMBER;
  g_requested_task = K230_TASK_NUMBER;
  g_task_switch_state = K230_TASK_SWITCH_IDLE;
  g_last_request_tick = 0U;
  g_last_valid_tick = 0U;
  g_result_generation = 0U;
  memset(&g_parser, 0, sizeof(g_parser));
  memset(&g_result, 0, sizeof(g_result));
  memset(&g_stats, 0, sizeof(g_stats));
  AppState_SetK230Online(0U);
  if (K230_LINK_RUNTIME_ENABLED != 0U) StartReceive();
}

void K230Link_HandleRxEvent(UART_HandleTypeDef *uart, uint16_t size)
{
  if ((K230_LINK_RUNTIME_ENABLED == 0U) || (uart != g_uart) || (size == 0U)) return;
  if (size > sizeof(g_rx_dma)) size = sizeof(g_rx_dma);
  RingPush(g_rx_dma, size);
  if (g_events != NULL) (void)osEventFlagsSet(g_events, K230_EVENT_RX_DATA);
  StartReceive();
}

void K230Link_HandleTxComplete(UART_HandleTypeDef *uart)
{
  if (uart == g_uart) g_tx_busy = 0U;
}

void K230Link_HandleError(UART_HandleTypeDef *uart)
{
  if (uart != g_uart) return;
  ++g_stats.uart_errors;
  StartReceive();
}

uint8_t K230Link_RequestOnce(uint16_t *sequence)
{
  uint16_t sent_sequence;
  if (g_request_pending || g_stream_active) return 0U;
  if (g_task_switch_queued || g_task_switch_pending ||
      !SendCommand(K230_CMD_REQUEST_ONCE, NULL, 0U, &sent_sequence)) return 0U;
  g_pending_sequence = sent_sequence;
  g_request_pending = 1U;
  g_request_deadline = HAL_GetTick() + K230_REQUEST_TIMEOUT_MS;
  g_last_request_tick = HAL_GetTick();
  if (sequence != NULL) *sequence = sent_sequence;
  return 1U;
}

uint8_t K230Link_StartStream(uint16_t *sequence)
{
  uint16_t sent_sequence;
  if (g_stream_active || g_task_switch_queued || g_task_switch_pending ||
      !SendCommand(K230_CMD_STREAM_START, NULL, 0U, &sent_sequence)) return 0U;
  g_stream_active = 1U;
  g_request_pending = 0U;
  if (sequence != NULL) *sequence = sent_sequence;
  return 1U;
}

uint8_t K230Link_StopStream(uint16_t *sequence)
{
  uint16_t sent_sequence;
  if (!g_stream_active || !SendCommand(K230_CMD_STREAM_STOP, NULL, 0U, &sent_sequence)) return 0U;
  g_stream_active = 0U;
  g_last_request_tick = HAL_GetTick();
  if (sequence != NULL) *sequence = sent_sequence;
  return 1U;
}

uint8_t K230Link_SelectTask(K230VisionTask task)
{
  uint32_t primask;
  if ((task != K230_TASK_NUMBER) && (task != K230_TASK_BEAN)) return 0U;

  primask = __get_PRIMASK();
  __disable_irq();
  g_requested_task = task;
  g_task_switch_queued = 1U;
  g_task_switch_pending = 0U;
  g_task_switch_state = K230_TASK_SWITCH_PENDING;
  g_request_pending = 0U;
  g_stream_active = 0U;
  __set_PRIMASK(primask);
  K230Link_InvalidateResult();
  return 1U;
}

K230VisionTask K230Link_GetSelectedTask(void)
{
  return g_selected_task;
}

K230VisionTask K230Link_GetRequestedTask(void)
{
  return g_requested_task;
}

K230TaskSwitchState K230Link_GetTaskSwitchState(void)
{
  return g_task_switch_state;
}

void K230Link_Task(void)
{
  uint8_t value;
  if (K230_LINK_RUNTIME_ENABLED == 0U)
  {
    for (;;) osDelay(1000U);
  }

  for (;;)
  {
    uint32_t now;
    (void)osEventFlagsWait(g_events, K230_EVENT_RX_DATA, osFlagsWaitAny, 20U);
    while (RingPop(&value)) ParserFeed(value);

    g_stats.rx_overflows = g_rx_overflows;
    now = HAL_GetTick();
    if (g_task_switch_queued && !g_tx_busy)
    {
      uint8_t task = (uint8_t)g_requested_task;
      uint16_t sequence;
      if (SendCommand(K230_CMD_SET_TASK, &task, 1U, &sequence))
      {
        g_task_switch_queued = 0U;
        g_task_switch_pending = 1U;
        g_task_switch_sequence = sequence;
        g_task_switch_deadline = now + K230_TASK_SWITCH_TIMEOUT_MS;
      }
    }
    if (g_task_switch_pending && ((int32_t)(now - g_task_switch_deadline) >= 0))
    {
      g_task_switch_pending = 0U;
      g_task_switch_state = K230_TASK_SWITCH_FAILED;
      AppState_SetK230Online(0U);
    }
    if (g_request_pending && ((int32_t)(now - g_request_deadline) >= 0))
    {
      g_request_pending = 0U;
      ++g_stats.request_timeouts;
    }
    if (!g_stream_active && !g_request_pending && !g_task_switch_queued &&
        !g_task_switch_pending &&
        ((uint32_t)(now - g_last_request_tick) >= K230_AUTO_REQUEST_PERIOD_MS))
    {
      (void)K230Link_RequestOnce(NULL);
    }
    if ((g_last_valid_tick == 0U) ||
        ((uint32_t)(now - g_last_valid_tick) > K230_ONLINE_TIMEOUT_MS))
    {
      AppState_SetK230Online(0U);
      /* K230流式发送固定20帧后会自动结束，超时后恢复5Hz单次请求。 */
      if (g_stream_active)
      {
        g_stream_active = 0U;
        g_last_request_tick = now;
      }
    }
  }
}

uint8_t K230Link_GetLatestResult(K230VisionResult *result)
{
  uint32_t before;
  uint32_t after;
  if (result == NULL) return 0U;
  do
  {
    before = g_result_generation;
    if (before & 1U) continue;
    __DMB();
    *result = g_result;
    __DMB();
    after = g_result_generation;
  } while ((before != after) || (after & 1U));
  return result->valid;
}

uint8_t K230Link_IsResultFresh(uint32_t maximum_age_ms)
{
  K230VisionResult result;
  return K230Link_GetLatestResult(&result) &&
         ((uint32_t)(HAL_GetTick() - result.received_tick_ms) <= maximum_age_ms);
}

void K230Link_InvalidateResult(void)
{
  K230VisionResult empty = {0};
  PublishResult(&empty);
}

void K230Link_GetStats(K230LinkStats *stats)
{
  if (stats == NULL) return;
  *stats = g_stats;
  stats->rx_overflows = g_rx_overflows;
}

uint8_t K230Link_TrySendText(const char *text)
{
  size_t length;
  uint8_t started = 0U;
  if ((g_uart == NULL) || (text == NULL) || (g_tx_mutex == NULL)) return 0U;
  if (osMutexAcquire(g_tx_mutex, 0U) != osOK) return 0U;
  if (!g_tx_busy)
  {
    length = strlen(text);
    if (length > sizeof(g_tx_buffer)) length = sizeof(g_tx_buffer);
    memcpy(g_tx_buffer, text, length);
    g_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(g_uart, g_tx_buffer, (uint16_t)length) == HAL_OK)
      started = 1U;
    else
      g_tx_busy = 0U;
  }
  (void)osMutexRelease(g_tx_mutex);
  return started;
}

void K230Link_SendText(const char *text)
{
  if (text == NULL) return;
  while (!K230Link_TrySendText(text)) osDelay(1U);
}

uint8_t K230Link_GetLatestDetection(K230Detection *detection)
{
  K230VisionResult result;
  if ((detection == NULL) || !K230Link_GetLatestResult(&result) || (result.count == 0U)) return 0U;
  detection->valid = 1U;
  detection->class_id = result.targets[0].semantic;
  detection->confidence_permille = (uint16_t)result.targets[0].confidence_percent * 10U;
  detection->center_x = (int16_t)result.targets[0].center_x;
  detection->center_y = (int16_t)result.targets[0].center_y;
  detection->frame_id = result.sequence;
  return 1U;
}
