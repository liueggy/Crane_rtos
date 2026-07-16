#include "infrared_remote.h"

#include "main.h"

/* TIM4 的 1 MHz 计数单位为微秒。以下窗口保留了接收头误差余量。 */
#define IR_LEADER_MARK_MIN_US 8000U
#define IR_LEADER_MARK_MAX_US 10000U
#define IR_LEADER_SPACE_MIN_US 3500U
#define IR_LEADER_SPACE_MAX_US 5500U
#define IR_BIT_MARK_MIN_US 350U
#define IR_BIT_MARK_MAX_US 850U
#define IR_BIT_ZERO_SPACE_MIN_US 350U
#define IR_BIT_ZERO_SPACE_MAX_US 900U
#define IR_BIT_ONE_SPACE_MIN_US 1300U
#define IR_BIT_ONE_SPACE_MAX_US 2100U
#define IR_FRAME_BITS 32U

typedef enum
{
  IR_WAIT_LEADER_MARK = 0,
  IR_WAIT_LEADER_SPACE,
  IR_WAIT_BIT_MARK,
  IR_WAIT_BIT_SPACE
} InfraredRemoteDecodeState;

static TIM_HandleTypeDef *g_ir_timer;
static volatile uint16_t g_last_capture;
static volatile uint32_t g_frame;
static volatile uint8_t g_bit_index;
static volatile uint8_t g_capture_started;
static volatile InfraredRemoteDecodeState g_decode_state;
static volatile uint8_t g_pending_command;
static volatile uint8_t g_command_ready;
static uint8_t g_last_command;

static uint8_t InfraredRemote_InRange(uint16_t value, uint16_t minimum, uint16_t maximum)
{
  return (value >= minimum) && (value <= maximum);
}

static void InfraredRemote_ResetFrame(void)
{
  g_frame = 0U;
  g_bit_index = 0U;
  g_decode_state = IR_WAIT_LEADER_MARK;
}

HAL_StatusTypeDef InfraredRemote_Init(TIM_HandleTypeDef *htim)
{
  if ((htim == NULL) || (htim->Instance != TIM4))
  {
    return HAL_ERROR;
  }

  g_ir_timer = htim;
  g_last_capture = 0U;
  g_frame = 0U;
  g_bit_index = 0U;
  g_capture_started = 0U;
  g_decode_state = IR_WAIT_LEADER_MARK;
  g_pending_command = 0U;
  g_command_ready = 0U;
  g_last_command = 0U;

  return HAL_TIM_IC_Start_IT(g_ir_timer, TIM_CHANNEL_4);
}

void InfraredRemote_HandleCapture(TIM_HandleTypeDef *htim)
{
  uint16_t capture;
  uint16_t interval_us;
  uint8_t command;
  uint8_t command_inverse;

  if ((htim != g_ir_timer) || (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_4))
  {
    return;
  }

  capture = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
  if (g_capture_started == 0U)
  {
    g_last_capture = capture;
    g_capture_started = 1U;
    return;
  }

  /* uint16_t 相减自然处理 TIM4 的一次计数器回绕。 */
  interval_us = (uint16_t)(capture - g_last_capture);
  g_last_capture = capture;

  switch (g_decode_state)
  {
    case IR_WAIT_LEADER_MARK:
      if (InfraredRemote_InRange(interval_us, IR_LEADER_MARK_MIN_US, IR_LEADER_MARK_MAX_US))
      {
        g_decode_state = IR_WAIT_LEADER_SPACE;
      }
      break;

    case IR_WAIT_LEADER_SPACE:
      if (InfraredRemote_InRange(interval_us, IR_LEADER_SPACE_MIN_US, IR_LEADER_SPACE_MAX_US))
      {
        InfraredRemote_ResetFrame();
        g_decode_state = IR_WAIT_BIT_MARK;
      }
      else
      {
        g_decode_state = IR_WAIT_LEADER_MARK;
      }
      break;

    case IR_WAIT_BIT_MARK:
      if (InfraredRemote_InRange(interval_us, IR_BIT_MARK_MIN_US, IR_BIT_MARK_MAX_US))
      {
        g_decode_state = IR_WAIT_BIT_SPACE;
      }
      else
      {
        g_decode_state = IR_WAIT_LEADER_MARK;
      }
      break;

    case IR_WAIT_BIT_SPACE:
      if (InfraredRemote_InRange(interval_us, IR_BIT_ZERO_SPACE_MIN_US, IR_BIT_ZERO_SPACE_MAX_US))
      {
        /* NEC 按低位在前发送，位序可直接写入对应索引。 */
      }
      else if (InfraredRemote_InRange(interval_us, IR_BIT_ONE_SPACE_MIN_US, IR_BIT_ONE_SPACE_MAX_US))
      {
        g_frame |= (1UL << g_bit_index);
      }
      else
      {
        g_decode_state = IR_WAIT_LEADER_MARK;
        break;
      }

      g_bit_index++;
      g_decode_state = IR_WAIT_BIT_MARK;
      if (g_bit_index == IR_FRAME_BITS)
      {
        command = (uint8_t)(g_frame >> 16U);
        command_inverse = (uint8_t)(g_frame >> 24U);
        if ((uint8_t)(command ^ command_inverse) == 0xFFU)
        {
          g_pending_command = command;
          g_command_ready = 1U;
        }
        InfraredRemote_ResetFrame();
        g_decode_state = IR_WAIT_LEADER_MARK;
      }
      break;

    default:
      g_decode_state = IR_WAIT_LEADER_MARK;
      break;
  }
}

void InfraredRemote_Process(void)
{
  uint8_t command;
  uint32_t primask;

  if (g_command_ready == 0U)
  {
    return;
  }

  /* 仅用极短临界区取走 ISR 给出的单条命令。 */
  primask = __get_PRIMASK();
  __disable_irq();
  command = g_pending_command;
  g_command_ready = 0U;
  __set_PRIMASK(primask);

  g_last_command = command;
  switch (command)
  {
    case IR_REMOTE_CMD_UP:
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      break;

    case IR_REMOTE_CMD_DOWN:
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      break;

    case IR_REMOTE_CMD_POWER:
      HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
      break;

    default:
      break;
  }
}

uint8_t InfraredRemote_GetLastCommand(void)
{
  return g_last_command;
}
