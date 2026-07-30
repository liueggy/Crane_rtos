#include "k230_link.h"

#include "app_state.h"
#include "cmsis_os2.h"

#include <string.h>

#define K230_ASCII_SELECT_NUMBER       ((uint8_t)'N')
#define K230_ASCII_SELECT_BEAN         ((uint8_t)'B')
#define K230_ASCII_ACK_NUMBER          ((uint8_t)'n')
#define K230_ASCII_ACK_BEAN            ((uint8_t)'b')
#define K230_RX_DMA_SIZE               128U
#define K230_RX_RING_SIZE              512U
#define K230_EVENT_RX_DATA             (1UL << 0)
#define K230_TASK_SWITCH_TIMEOUT_MS    10000U
#define K230_COMPAT_CONFIDENCE_PERCENT 100U
#define K230_COMPAT_CENTER_X           320U
#define K230_COMPAT_CENTER_Y           240U

static UART_HandleTypeDef *g_uart;
static osEventFlagsId_t g_events;
static osMutexId_t g_tx_mutex;
static uint8_t g_rx_dma[K230_RX_DMA_SIZE];
static uint8_t g_rx_ring[K230_RX_RING_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint32_t g_rx_overflows;
static uint8_t g_tx_byte;
static volatile uint8_t g_tx_busy;
static volatile uint8_t g_task_switch_queued;
static volatile uint8_t g_task_switch_pending;
static volatile K230VisionTask g_selected_task;
static volatile K230VisionTask g_requested_task;
static volatile K230TaskSwitchState g_task_switch_state;
static uint32_t g_task_switch_deadline;
static uint32_t g_last_valid_tick;
static uint16_t g_next_result_sequence;
static K230VisionResult g_result;
static volatile uint32_t g_result_generation;
static K230LinkStats g_stats;

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

static void PublishSemantic(uint8_t semantic, K230VisionTask task)
{
  K230VisionResult result = {0};
  result.valid = 1U;
  result.task = (uint8_t)task;
  result.count = 1U;
  result.sequence = g_next_result_sequence++;
  result.received_tick_ms = HAL_GetTick();
  result.targets[0].semantic = semantic;
  /* 新字符协议不含置信度和坐标，保留兼容占位值供旧显示结构使用。 */
  result.targets[0].confidence_percent = K230_COMPAT_CONFIDENCE_PERCENT;
  result.targets[0].center_x = K230_COMPAT_CENTER_X;
  result.targets[0].center_y = K230_COMPAT_CENTER_Y;
  PublishResult(&result);
  ++g_stats.valid_frames;
  g_last_valid_tick = result.received_tick_ms;
  AppState_SetK230Online(1U);
}

static uint8_t TrySendByte(uint8_t value)
{
  uint8_t started = 0U;
  if ((g_uart == NULL) || (g_tx_mutex == NULL)) return 0U;
  if (osMutexAcquire(g_tx_mutex, 0U) != osOK) return 0U;
  if (!g_tx_busy)
  {
    g_tx_byte = value;
    g_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(g_uart, &g_tx_byte, 1U) == HAL_OK)
      started = 1U;
    else
      g_tx_busy = 0U;
  }
  (void)osMutexRelease(g_tx_mutex);
  return started;
}

static void HandleTaskAck(uint8_t value)
{
  K230VisionTask acknowledged_task =
      (value == K230_ASCII_ACK_NUMBER) ? K230_TASK_NUMBER : K230_TASK_BEAN;
  if (!g_task_switch_pending || (acknowledged_task != g_requested_task)) return;
  g_task_switch_pending = 0U;
  g_selected_task = acknowledged_task;
  g_task_switch_state = K230_TASK_SWITCH_IDLE;
  g_last_valid_tick = HAL_GetTick();
  K230Link_InvalidateResult();
  AppState_SetK230Online(1U);
}

static void HandleRxByte(uint8_t value)
{
  uint8_t semantic = K230_SEMANTIC_UNKNOWN;

  if ((value == K230_ASCII_ACK_NUMBER) || (value == K230_ASCII_ACK_BEAN))
  {
    HandleTaskAck(value);
    return;
  }

  /* 切换确认前丢弃旧模型仍在途的识别字符。 */
  if (g_task_switch_queued || g_task_switch_pending) return;

  if ((g_selected_task == K230_TASK_NUMBER) &&
      (value >= (uint8_t)'1') && (value <= (uint8_t)'5'))
  {
    semantic = (uint8_t)(value - (uint8_t)'0');
  }
  else if (g_selected_task == K230_TASK_BEAN)
  {
    semantic = (value == (uint8_t)'L') ? K230_SEMANTIC_BEAN_L :
               (value == (uint8_t)'H') ? K230_SEMANTIC_BEAN_H :
               (value == (uint8_t)'B') ? K230_SEMANTIC_BEAN_B :
                                         K230_SEMANTIC_UNKNOWN;
  }

  if (semantic != K230_SEMANTIC_UNKNOWN)
    PublishSemantic(semantic, g_selected_task);
  else
    ++g_stats.format_errors;
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
  g_task_switch_queued = 0U;
  g_task_switch_pending = 0U;
  g_selected_task = K230_TASK_NUMBER;
  g_requested_task = K230_TASK_NUMBER;
  g_task_switch_state = K230_TASK_SWITCH_IDLE;
  g_task_switch_deadline = 0U;
  g_last_valid_tick = 0U;
  g_next_result_sequence = 1U;
  g_result_generation = 0U;
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
  AppState_SetK230Online(0U);
  StartReceive();
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
    while (RingPop(&value)) HandleRxByte(value);

    g_stats.rx_overflows = g_rx_overflows;
    now = HAL_GetTick();
    if (g_task_switch_queued && !g_tx_busy)
    {
      uint8_t command = (g_requested_task == K230_TASK_NUMBER) ?
                        K230_ASCII_SELECT_NUMBER : K230_ASCII_SELECT_BEAN;
      if (TrySendByte(command))
      {
        g_task_switch_queued = 0U;
        g_task_switch_pending = 1U;
        g_task_switch_deadline = now + K230_TASK_SWITCH_TIMEOUT_MS;
      }
    }
    if (g_task_switch_pending && ((int32_t)(now - g_task_switch_deadline) >= 0))
    {
      g_task_switch_pending = 0U;
      g_task_switch_state = K230_TASK_SWITCH_FAILED;
      ++g_stats.request_timeouts;
      AppState_SetK230Online(0U);
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

uint8_t K230Link_GetLatestDetection(K230Detection *detection)
{
  K230VisionResult result;
  if ((detection == NULL) || !K230Link_GetLatestResult(&result) ||
      (result.count == 0U)) return 0U;
  detection->valid = 1U;
  detection->class_id = result.targets[0].semantic;
  detection->confidence_permille = 1000U;
  detection->center_x = (int16_t)K230_COMPAT_CENTER_X;
  detection->center_y = (int16_t)K230_COMPAT_CENTER_Y;
  detection->frame_id = result.sequence;
  return 1U;
}
