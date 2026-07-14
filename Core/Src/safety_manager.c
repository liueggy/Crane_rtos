#include "safety_manager.h"

#include "app_state.h"
#include "chassis_motion.h"
#include "main.h"
#include "stepper_axis.h"

static volatile uint32_t g_flags;

void SafetyManager_Init(void)
{
  g_flags = 0U;
}

void SafetyManager_TriggerEstop(void)
{
  g_flags |= SAFETY_FAULT_ESTOP;
  AppState_SetEstopActive(1U);
  AppState_SetFaultFlags(g_flags);
  ChassisMotion_Stop();
  StepperAxis_StopAll();
}

void SafetyManager_HandleExti(uint16_t gpio_pin)
{
  uint32_t limit = 0U;
  if (gpio_pin == LIMIT_X_MIN_Pin) limit = SAFETY_LIMIT_X_MIN;
  else if (gpio_pin == LIMIT_X_MAX_Pin) limit = SAFETY_LIMIT_X_MAX;
  else if (gpio_pin == LIMIT_Z_MAX_Pin) limit = SAFETY_LIMIT_Z_MAX;
#ifdef LIMIT_Z_MIN_Pin
  else if (gpio_pin == LIMIT_Z_MIN_Pin) limit = SAFETY_LIMIT_Z_MIN;
#endif
  if (limit != 0U)
  {
    g_flags |= limit | SAFETY_FAULT_LIMIT;
    AppState_SetFaultFlags(g_flags);
    StepperAxis_StopAll();
  }
}

uint32_t SafetyManager_GetFlags(void)
{
  return g_flags;
}

uint8_t SafetyManager_IsEstopActive(void)
{
  return (g_flags & SAFETY_FAULT_ESTOP) != 0U;
}
