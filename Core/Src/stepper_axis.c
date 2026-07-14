#include "stepper_axis.h"

#include "app_state.h"
#include "cmsis_os2.h"
#include "main.h"

#define STEPPER_RUN_PULSE 250U

static TIM_HandleTypeDef *g_timer;
static uint8_t g_enabled[STEPPER_AXIS_COUNT];
static uint8_t g_reverse[STEPPER_AXIS_COUNT];

static uint32_t AxisChannel(StepperAxisId axis)
{
  return (axis == STEPPER_AXIS_X) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
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
  StepperAxis_StopAll();
}

void StepperAxis_SetEnabled(StepperAxisId axis, uint8_t enabled)
{
  if ((g_timer == NULL) || (axis >= STEPPER_AXIS_COUNT)) return;
  g_enabled[axis] = enabled ? 1U : 0U;
  WriteEnable(axis, g_enabled[axis]);
  __HAL_TIM_SET_COMPARE(g_timer, AxisChannel(axis),
                        g_enabled[axis] ? STEPPER_RUN_PULSE : 0U);
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
  StepperAxis_SetEnabled(STEPPER_AXIS_X, 0U);
  StepperAxis_SetEnabled(STEPPER_AXIS_Z, 0U);
}

void StepperAxis_UpdateTelemetry(void)
{
  AppState_SetStepperTelemetry(g_enabled[STEPPER_AXIS_Z], g_reverse[STEPPER_AXIS_Z],
                               g_enabled[STEPPER_AXIS_Z] ? STEPPER_RUN_PULSE : 0U);
}
