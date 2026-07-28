#include "stepper_axis.h"

#include "app_config.h"
#include "app_state.h"
#include "cmsis_os2.h"
#include "main.h"
#include "photo_sensor.h"

#define STEPPER_RUN_PULSE 250U

static TIM_HandleTypeDef *g_timer;
static uint8_t g_enabled[STEPPER_AXIS_COUNT];
static uint8_t g_reverse[STEPPER_AXIS_COUNT];
static uint8_t g_hold_when_stopped[STEPPER_AXIS_COUNT];
static uint8_t g_has_run[STEPPER_AXIS_COUNT];
static uint8_t g_holding[STEPPER_AXIS_COUNT];
static volatile uint8_t g_pulse_move_active[STEPPER_AXIS_COUNT];
static volatile uint32_t g_commanded_pulses[STEPPER_AXIS_COUNT];
static volatile uint32_t g_completed_pulses[STEPPER_AXIS_COUNT];
static volatile uint32_t g_remaining_pulses[STEPPER_AXIS_COUNT];
static volatile int32_t g_position_pulses[STEPPER_AXIS_COUNT];
static uint8_t g_photo_limit_valid;
static StepperAxisId g_photo_limit_axis;
static uint8_t g_photo_limit_reverse;

static uint32_t AxisChannel(StepperAxisId axis)
{
  return (axis == STEPPER_AXIS_X) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
}

static uint32_t AxisInterrupt(StepperAxisId axis)
{
  return (axis == STEPPER_AXIS_X) ? TIM_IT_CC1 : TIM_IT_CC2;
}

static uint32_t AxisMaximumMovePulses(StepperAxisId axis)
{
  return (axis == STEPPER_AXIS_X) ? STEPPER_X_TRAVEL_PULSES :
                                    STEPPER_Z_TRAVEL_PULSES;
}

static uint8_t CanStart(StepperAxisId axis)
{
  StepperAxisId other = (axis == STEPPER_AXIS_X) ? STEPPER_AXIS_Z : STEPPER_AXIS_X;
  /* X/Z共用同一个光电门，禁止双轴同时运动导致限位归属不确定。 */
  if (g_enabled[other]) return 0U;
  if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) == 0U) return 1U;
  return g_photo_limit_valid && (axis == g_photo_limit_axis) &&
         (g_reverse[axis] != g_photo_limit_reverse);
}

static void CancelPulseMove(StepperAxisId axis)
{
  if ((g_timer == NULL) || (axis >= STEPPER_AXIS_COUNT)) return;
  if (g_pulse_move_active[axis])
  {
    (void)HAL_TIM_PWM_Stop_IT(g_timer, AxisChannel(axis));
    __HAL_TIM_SET_COMPARE(g_timer, AxisChannel(axis), 0U);
  }
  __HAL_TIM_DISABLE_IT(g_timer, AxisInterrupt(axis));
  g_pulse_move_active[axis] = 0U;
  g_remaining_pulses[axis] = 0U;
}

static void WriteEnable(StepperAxisId axis, uint8_t enabled)
{
  if (axis == STEPPER_AXIS_X)
  {
    HAL_GPIO_WritePin(STEP_X_ENA_GPIO_Port, STEP_X_ENA_Pin,
                      enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(STEP_Z_ENA_GPIO_Port, STEP_Z_ENA_Pin,
                      enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
}

void StepperAxis_Init(TIM_HandleTypeDef *timer)
{
  g_timer = timer;
  g_photo_limit_valid = 0U;
  g_photo_limit_axis = STEPPER_AXIS_X;
  g_photo_limit_reverse = 0U;
  g_hold_when_stopped[STEPPER_AXIS_X] = 0U;
  g_hold_when_stopped[STEPPER_AXIS_Z] = 1U;
  for (uint8_t axis = 0U; axis < STEPPER_AXIS_COUNT; ++axis)
  {
    g_has_run[axis] = 0U;
    g_holding[axis] = 0U;
    g_pulse_move_active[axis] = 0U;
    g_commanded_pulses[axis] = 0U;
    g_completed_pulses[axis] = 0U;
    g_remaining_pulses[axis] = 0U;
    g_position_pulses[axis] = 0;
  }
  StepperAxis_StopAll();
  /* Z轴承受夹爪重力：初始化后立即使能保持，不输出脉冲。 */
  g_has_run[STEPPER_AXIS_Z] = 1U;
  g_holding[STEPPER_AXIS_Z] = 1U;
  WriteEnable(STEPPER_AXIS_Z, 1U);
}

void StepperAxis_SetEnabled(StepperAxisId axis, uint8_t enabled)
{
  HAL_StatusTypeDef status;
  uint32_t channel;

  if ((g_timer == NULL) || (axis >= STEPPER_AXIS_COUNT)) return;
  CancelPulseMove(axis);
  if (enabled && !CanStart(axis))
  {
    enabled = 0U;
  }
  channel = AxisChannel(axis);
  g_enabled[axis] = enabled ? 1U : 0U;
  if (g_enabled[axis])
  {
    g_has_run[axis] = 1U;
    g_holding[axis] = 0U;
    WriteEnable(axis, 1U);
    (void)HAL_TIM_PWM_Stop(g_timer, channel);
    __HAL_TIM_SET_COMPARE(g_timer, channel, STEPPER_RUN_PULSE);
    __HAL_TIM_CLEAR_IT(g_timer, AxisInterrupt(axis));
    status = HAL_TIM_PWM_Start_IT(g_timer, channel);
    if (status != HAL_OK)
    {
      g_enabled[axis] = 0U;
      g_holding[axis] = (g_hold_when_stopped[axis] && g_has_run[axis]) ? 1U : 0U;
      WriteEnable(axis, g_holding[axis]);
      __HAL_TIM_SET_COMPARE(g_timer, channel, 0U);
    }
  }
  else
  {
    (void)HAL_TIM_PWM_Stop_IT(g_timer, channel);
    __HAL_TIM_SET_COMPARE(g_timer, channel, 0U);
    g_holding[axis] = (g_hold_when_stopped[axis] && g_has_run[axis]) ? 1U : 0U;
    WriteEnable(axis, g_holding[axis]);
  }
}

HAL_StatusTypeDef StepperAxis_MovePulses(StepperAxisId axis,
                                         uint32_t pulse_count,
                                         uint8_t reverse)
{
  HAL_StatusTypeDef status;
  uint32_t channel;

  if ((g_timer == NULL) || (axis >= STEPPER_AXIS_COUNT) || (pulse_count == 0U) ||
      (pulse_count > AxisMaximumMovePulses(axis)))
    return HAL_ERROR;

  StepperAxis_SetEnabled(axis, 0U);
  StepperAxis_SetDirectionReverse(axis, reverse);
  if (!CanStart(axis)) return HAL_BUSY;

  channel = AxisChannel(axis);
  /* 主程序已用普通模式启动PWM，先复位HAL通道状态再切到CC中断模式。 */
  (void)HAL_TIM_PWM_Stop(g_timer, channel);
  g_commanded_pulses[axis] = pulse_count;
  g_completed_pulses[axis] = 0U;
  g_remaining_pulses[axis] = pulse_count;
  g_pulse_move_active[axis] = 1U;
  g_enabled[axis] = 1U;
  g_has_run[axis] = 1U;
  g_holding[axis] = 0U;
  WriteEnable(axis, 1U);

  /* 从周期边界装载CCR，随后用CC中断在每个完整脉冲下降沿计数。 */
  __HAL_TIM_SET_COMPARE(g_timer, channel, STEPPER_RUN_PULSE);
  __HAL_TIM_SET_COUNTER(g_timer, 0U);
  if (HAL_TIM_GenerateEvent(g_timer, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
  {
    g_pulse_move_active[axis] = 0U;
    g_remaining_pulses[axis] = 0U;
    StepperAxis_SetEnabled(axis, 0U);
    return HAL_ERROR;
  }
  __HAL_TIM_CLEAR_IT(g_timer, AxisInterrupt(axis));
  status = HAL_TIM_PWM_Start_IT(g_timer, channel);
  if (status != HAL_OK)
  {
    g_pulse_move_active[axis] = 0U;
    g_remaining_pulses[axis] = 0U;
    StepperAxis_SetEnabled(axis, 0U);
  }
  return status;
}

void StepperAxis_HandlePulseFinished(TIM_HandleTypeDef *timer)
{
  StepperAxisId axis;
  uint32_t channel;

  if ((timer != g_timer) || (timer->Instance != TIM1)) return;
  if (timer->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    axis = STEPPER_AXIS_X;
  else if (timer->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    axis = STEPPER_AXIS_Z;
  else
    return;

  if (!g_enabled[axis]) return;
  /* 统一坐标符号：X正转向左为+X，Z正转向下为+Z。 */
  g_position_pulses[axis] += g_reverse[axis] ? -1 : 1;
  if (!g_pulse_move_active[axis] || (g_remaining_pulses[axis] == 0U)) return;
  --g_remaining_pulses[axis];
  ++g_completed_pulses[axis];
  if (g_remaining_pulses[axis] != 0U) return;

  channel = AxisChannel(axis);
  (void)HAL_TIM_PWM_Stop_IT(g_timer, channel);
  __HAL_TIM_SET_COMPARE(g_timer, channel, 0U);
  g_pulse_move_active[axis] = 0U;
  g_enabled[axis] = 0U;
  g_holding[axis] = (g_hold_when_stopped[axis] && g_has_run[axis]) ? 1U : 0U;
  WriteEnable(axis, g_holding[axis]);
}

uint8_t StepperAxis_IsPulseMoveActive(StepperAxisId axis)
{
  return (axis < STEPPER_AXIS_COUNT) ? g_pulse_move_active[axis] : 0U;
}

uint32_t StepperAxis_GetCommandedPulses(StepperAxisId axis)
{
  return (axis < STEPPER_AXIS_COUNT) ? g_commanded_pulses[axis] : 0U;
}

uint32_t StepperAxis_GetCompletedPulses(StepperAxisId axis)
{
  return (axis < STEPPER_AXIS_COUNT) ? g_completed_pulses[axis] : 0U;
}

uint32_t StepperAxis_GetRemainingPulses(StepperAxisId axis)
{
  return (axis < STEPPER_AXIS_COUNT) ? g_remaining_pulses[axis] : 0U;
}

int32_t StepperAxis_GetPositionPulses(StepperAxisId axis)
{
  return (axis < STEPPER_AXIS_COUNT) ? g_position_pulses[axis] : 0;
}

uint8_t StepperAxis_ResetPositionPulses(StepperAxisId axis)
{
  if ((axis >= STEPPER_AXIS_COUNT) || g_enabled[axis]) return 0U;
  g_position_pulses[axis] = 0;
  return 1U;
}

uint8_t StepperAxis_SetPositionPulses(StepperAxisId axis, int32_t position)
{
  if ((axis >= STEPPER_AXIS_COUNT) || g_enabled[axis]) return 0U;
  g_position_pulses[axis] = position;
  return 1U;
}

uint8_t StepperAxis_IsEnabled(StepperAxisId axis)
{
  return (axis < STEPPER_AXIS_COUNT) ? g_enabled[axis] : 0U;
}

uint8_t StepperAxis_IsHolding(StepperAxisId axis)
{
  return (axis < STEPPER_AXIS_COUNT) ? g_holding[axis] : 0U;
}

void StepperAxis_SetHoldWhenStopped(StepperAxisId axis, uint8_t enabled)
{
  if (axis >= STEPPER_AXIS_COUNT) return;
  g_hold_when_stopped[axis] = enabled ? 1U : 0U;
  if (!g_enabled[axis])
  {
    /* 开启静态保持时立即吸合，不要求该轴已经运行过。 */
    g_holding[axis] = g_hold_when_stopped[axis] ? 1U : 0U;
    if (g_holding[axis]) g_has_run[axis] = 1U;
    WriteEnable(axis, g_holding[axis]);
  }
}

uint8_t StepperAxis_ArmPhotoLimitEscape(StepperAxisId axis,
                                        uint8_t blocked_direction_reverse)
{
  if ((axis >= STEPPER_AXIS_COUNT) ||
      (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) == 0U) ||
      g_enabled[STEPPER_AXIS_X] || g_enabled[STEPPER_AXIS_Z]) return 0U;
  g_photo_limit_axis = axis;
  g_photo_limit_reverse = blocked_direction_reverse ? 1U : 0U;
  g_photo_limit_valid = 1U;
  return 1U;
}

uint8_t StepperAxis_IsPhotoLimitOwnedBy(StepperAxisId axis)
{
  return (axis < STEPPER_AXIS_COUNT) &&
         (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U) &&
         g_photo_limit_valid && (g_photo_limit_axis == axis);
}

void StepperAxis_ProcessPhotoInterlock(void)
{
  if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) == 0U)
  {
    g_photo_limit_valid = 0U;
    return;
  }

  if (!g_photo_limit_valid)
  {
    /* 遥控调试采用单轴互锁，因此正常情况下只会命中一个运动轴。 */
    for (uint8_t axis = 0U; axis < STEPPER_AXIS_COUNT; ++axis)
    {
      if (g_enabled[axis])
      {
        g_photo_limit_axis = (StepperAxisId)axis;
        g_photo_limit_reverse = g_reverse[axis];
        g_photo_limit_valid = 1U;
        StepperAxis_SetEnabled((StepperAxisId)axis, 0U);
        break;
      }
    }
  }
  else if (g_enabled[g_photo_limit_axis] &&
           (g_reverse[g_photo_limit_axis] == g_photo_limit_reverse))
  {
    StepperAxis_SetEnabled(g_photo_limit_axis, 0U);
  }
}

void StepperAxis_SetDirectionReverse(StepperAxisId axis, uint8_t reverse)
{
  if (axis >= STEPPER_AXIS_COUNT) return;
  if (g_enabled[axis])
  {
    __HAL_TIM_SET_COMPARE(g_timer, AxisChannel(axis), 0U);
    osDelay(2U);
  }
  g_reverse[axis] = reverse ? 1U : 0U;
  if (axis == STEPPER_AXIS_X)
  {
    HAL_GPIO_WritePin(STEP_X_DIR_GPIO_Port, STEP_X_DIR_Pin,
                      g_reverse[axis] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(STEP_Z_DIR_GPIO_Port, STEP_Z_DIR_Pin,
                      g_reverse[axis] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  if (g_enabled[axis]) __HAL_TIM_SET_COMPARE(g_timer, AxisChannel(axis), STEPPER_RUN_PULSE);
}

void StepperAxis_ToggleDirection(StepperAxisId axis)
{
  if (axis < STEPPER_AXIS_COUNT) StepperAxis_SetDirectionReverse(axis, !g_reverse[axis]);
}

void StepperAxis_StopAll(void)
{
  for (uint8_t axis = 0U; axis < STEPPER_AXIS_COUNT; ++axis)
  {
    g_enabled[axis] = 0U;
    g_holding[axis] = 0U;
    g_has_run[axis] = 0U;
    CancelPulseMove((StepperAxisId)axis);
    WriteEnable((StepperAxisId)axis, 0U);
    if (g_timer != NULL)
      __HAL_TIM_SET_COMPARE(g_timer, AxisChannel((StepperAxisId)axis), 0U);
  }
}

void StepperAxis_UpdateTelemetry(void)
{
  AppState_SetStepperTelemetry(g_enabled[STEPPER_AXIS_Z], g_reverse[STEPPER_AXIS_Z],
                               g_enabled[STEPPER_AXIS_Z] ? STEPPER_RUN_PULSE : 0U);
}
