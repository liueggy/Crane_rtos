#include "app_state.h"

#include "cmsis_gcc.h"
#include <string.h>

/* 所有任务共享的整机状态；对外只通过本文件的接口访问。 */
static AppState g_state;

void AppState_Init(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  memset(&g_state, 0, sizeof(g_state));
  g_state.mode = APP_MODE_IDLE;
  __set_PRIMASK(primask);
}

void AppState_GetSnapshot(AppState *snapshot)
{
  uint32_t primask;
  if (snapshot == NULL)
  {
    return;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  memcpy(snapshot, &g_state, sizeof(*snapshot));
  __set_PRIMASK(primask);
}

void AppState_SetRunEnabled(uint8_t enabled)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_state.run_enabled = enabled ? 1U : 0U;
  /* 当前阶段按键控制属于手动模式；自动状态机接入后再扩展这里。 */
  g_state.mode = enabled ? APP_MODE_MANUAL : APP_MODE_IDLE;
  __set_PRIMASK(primask);
}

uint8_t AppState_GetRunEnabled(void)
{
  uint8_t enabled;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  enabled = g_state.run_enabled;
  __set_PRIMASK(primask);
  return enabled;
}

void AppState_SetEstopActive(uint8_t active)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_state.estop_active = active ? 1U : 0U;
  if (active)
  {
    g_state.run_enabled = 0U;
    g_state.mode = APP_MODE_FAULT;
  }
  __set_PRIMASK(primask);
}

void AppState_SetMode(AppMode mode)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_state.mode = mode;
  __set_PRIMASK(primask);
}

void AppState_SetK230Online(uint8_t online)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_state.k230_online = online ? 1U : 0U;
  __set_PRIMASK(primask);
}

void AppState_SetFaultFlags(uint32_t flags)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_state.fault_flags = flags;
  __set_PRIMASK(primask);
}

void AppState_SetUiPage(uint8_t page)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_state.ui_page = page;
  __set_PRIMASK(primask);
}

void AppState_SetStepperTelemetry(uint8_t enabled, uint8_t direction_reverse,
                                  uint16_t pulse)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_state.stepper_enabled = enabled ? 1U : 0U;
  g_state.stepper_direction_reverse = direction_reverse ? 1U : 0U;
  g_state.stepper_pulse = pulse;
  __set_PRIMASK(primask);
}

void AppState_SetMotorTelemetry(uint8_t index, int32_t count, float target_rpm,
                                float measured_rpm, int16_t pwm_command)
{
  uint32_t primask;
  if (index >= APP_MOTOR_COUNT)
  {
    return;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  g_state.encoder_count[index] = count;
  g_state.target_rpm[index] = target_rpm;
  g_state.measured_rpm[index] = measured_rpm;
  g_state.pwm_command[index] = pwm_command;
  __set_PRIMASK(primask);
}
