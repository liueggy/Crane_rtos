#include "infrared_remote.h"

#include "main.h"
#include "buzzer.h"
#include "chassis_motion.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "ui_manager.h"

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
#define IR_X_DIRECTION_CHANGE_DELAY_MS 300U
#define IR_KEY_BEEP_DURATION_MS 60U

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
static volatile InfraredControlAxis g_selected_axis;
static volatile InfraredMotionState g_motion_state[IR_CONTROL_AXIS_COUNT];
static InfraredMotionState g_selected_direction[IR_CONTROL_AXIS_COUNT];
static uint8_t g_direction_change_pending[IR_CONTROL_AXIS_COUNT];
static uint32_t g_direction_resume_tick[IR_CONTROL_AXIS_COUNT];
static int8_t InfraredRemote_GetDcMotorTestIndex(uint8_t command)
{
  switch (command)
  {
    case IR_REMOTE_CMD_1: return 0;
    case IR_REMOTE_CMD_2: return 1;
    case IR_REMOTE_CMD_3: return 2;
    case IR_REMOTE_CMD_4: return 3;
    default: return -1;
  }
}

static StepperAxisId InfraredRemote_ToStepperAxis(InfraredControlAxis axis)
{
  return (axis == IR_CONTROL_AXIS_X) ? STEPPER_AXIS_X : STEPPER_AXIS_Z;
}

static void InfraredRemote_ApplyMotion(InfraredControlAxis axis, InfraredMotionState state)
{
  StepperAxisId stepper_axis = InfraredRemote_ToStepperAxis(axis);

  if (state == IR_MOTION_STOP)
  {
    StepperAxis_SetEnabled(stepper_axis, 0U);
    g_motion_state[axis] = IR_MOTION_STOP;
    return;
  }

  /* 启动前先在无脉冲状态写入方向，避免 DIR 与首个脉冲距离过短。 */
  StepperAxis_SetEnabled(stepper_axis, 0U);
  StepperAxis_SetDirectionReverse(stepper_axis, (state == IR_MOTION_REVERSE) ? 1U : 0U);
  StepperAxis_SetEnabled(stepper_axis, 1U);
  g_motion_state[axis] = state;
}

static void InfraredRemote_SelectAxis(InfraredControlAxis axis)
{
  if (axis == g_selected_axis) return;

  /* 手动调试只允许单轴运动，切轴时先关闭原轴。 */
  InfraredRemote_ApplyMotion(g_selected_axis, IR_MOTION_STOP);
  g_direction_change_pending[g_selected_axis] = 0U;
  g_selected_axis = axis;
}

static void InfraredRemote_SelectDirection(InfraredControlAxis axis,
                                           InfraredMotionState direction)
{
  InfraredRemote_SelectAxis(axis);
  if ((g_motion_state[axis] != IR_MOTION_STOP) &&
      (g_motion_state[axis] != direction))
  {
    InfraredRemote_ApplyMotion(axis, IR_MOTION_STOP);
    g_direction_change_pending[axis] = 1U;
    g_direction_resume_tick[axis] = HAL_GetTick() + IR_X_DIRECTION_CHANGE_DELAY_MS;
  }
  else if (g_direction_change_pending[axis] != 0U)
  {
    g_direction_resume_tick[axis] = HAL_GetTick() + IR_X_DIRECTION_CHANGE_DELAY_MS;
  }
  g_selected_direction[axis] = direction;
}

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
  g_selected_axis = IR_CONTROL_AXIS_X;
  for (uint8_t axis = 0U; axis < IR_CONTROL_AXIS_COUNT; ++axis)
  {
    g_motion_state[axis] = IR_MOTION_STOP;
    g_selected_direction[axis] = IR_MOTION_FORWARD;
    g_direction_change_pending[axis] = 0U;
    g_direction_resume_tick[axis] = 0U;
    InfraredRemote_ApplyMotion((InfraredControlAxis)axis, IR_MOTION_STOP);
  }

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
    /* 翻转捕获极性，使下一次捕获在相反的边沿触发。 */
    g_ir_timer->Instance->CCER ^= TIM_CCER_CC4P;
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

  /* 翻转捕获极性，使下一次捕获在相反的边沿触发。
   * TIM4（通用定时器）无硬件 BOTHEDGE 支持，需软件交替。 */
  g_ir_timer->Instance->CCER ^= TIM_CCER_CC4P;
}

void InfraredRemote_Process(void)
{
  uint8_t command;
  uint8_t safety_fault;
  int8_t dc_motor_test_index;
  uint32_t primask;

  /* 急停或限位故障优先级最高，禁止待执行命令重新启动 X 轴。 */
  safety_fault = ((SafetyManager_GetFlags() &
                  (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) != 0U) ? 1U : 0U;
  if (safety_fault != 0U)
  {
    for (uint8_t axis = 0U; axis < IR_CONTROL_AXIS_COUNT; ++axis)
    {
      InfraredRemote_ApplyMotion((InfraredControlAxis)axis, IR_MOTION_STOP);
      g_direction_change_pending[axis] = 0U;
    }
  }

  /* 换向采用“先停机、等待死区、再恢复”的非阻塞状态机。 */
  if ((safety_fault == 0U) &&
      (g_direction_change_pending[g_selected_axis] != 0U) &&
      ((int32_t)(HAL_GetTick() - g_direction_resume_tick[g_selected_axis]) >= 0))
  {
    InfraredRemote_ApplyMotion(g_selected_axis, g_selected_direction[g_selected_axis]);
    g_direction_change_pending[g_selected_axis] = 0U;
  }

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
  Buzzer_Beep(IR_KEY_BEEP_DURATION_MS);

  /* 故障状态只确认收到按键，不执行任何电机动作。 */
  if (safety_fault != 0U) return;

  /* 数字 1~4 交给底盘任务执行，避免与 10ms 电机控制周期争用 PWM。 */
  dc_motor_test_index = InfraredRemote_GetDcMotorTestIndex(command);
  if (dc_motor_test_index >= 0)
  {
    uint8_t index = (uint8_t)dc_motor_test_index;
    ChassisMotion_SetRemoteMotorEnabled(
        index, ChassisMotion_IsRemoteMotorEnabled(index) ? 0U : 1U);
    UiManager_SetPage(UI_PAGE_MOTOR_SPEED);
    return;
  }
  if (command == IR_REMOTE_CMD_5)
  {
    ChassisMotion_ToggleRemoteAllMotors();
    UiManager_SetPage(UI_PAGE_MOTOR_SPEED);
    return;
  }
  if (command == IR_REMOTE_CMD_6)
  {
    (void)ChassisMotion_ToggleRemoteDirection();
    UiManager_SetPage(UI_PAGE_MOTOR_SPEED);
    return;
  }

  if (command == IR_REMOTE_CMD_UP)
  {
    ServoControl_AdjustAngle(0U, 45);
    UiManager_SetPage(UI_PAGE_SERVO);
  }
  else if (command == IR_REMOTE_CMD_DOWN)
  {
    ServoControl_AdjustAngle(0U, -45);
    UiManager_SetPage(UI_PAGE_SERVO);
  }
  else if (command == IR_REMOTE_CMD_RIGHT)
  {
    InfraredRemote_SelectDirection(IR_CONTROL_AXIS_X, IR_MOTION_FORWARD);
  }
  else if (command == IR_REMOTE_CMD_LEFT)
  {
    InfraredRemote_SelectDirection(IR_CONTROL_AXIS_X, IR_MOTION_REVERSE);
  }
  else if (command == IR_REMOTE_CMD_POWER)
  {
    if (g_direction_change_pending[g_selected_axis] != 0U)
    {
      g_direction_change_pending[g_selected_axis] = 0U;
      InfraredRemote_ApplyMotion(g_selected_axis, IR_MOTION_STOP);
    }
    else
    {
      InfraredRemote_ApplyMotion(g_selected_axis,
          (g_motion_state[g_selected_axis] == IR_MOTION_STOP) ?
          g_selected_direction[g_selected_axis] : IR_MOTION_STOP);
    }
  }
}

uint8_t InfraredRemote_GetLastCommand(void)
{
  return g_last_command;
}

InfraredControlAxis InfraredRemote_GetSelectedAxis(void)
{
  return g_selected_axis;
}

InfraredMotionState InfraredRemote_GetSelectedMotionState(void)
{
  return g_motion_state[g_selected_axis];
}

uint8_t InfraredRemote_GetSelectedDirectionReverse(void)
{
  return (g_selected_direction[g_selected_axis] == IR_MOTION_REVERSE) ? 1U : 0U;
}

uint8_t InfraredRemote_IsDirectionChangePending(void)
{
  return g_direction_change_pending[g_selected_axis];
}
