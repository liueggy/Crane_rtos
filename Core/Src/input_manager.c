#include "input_manager.h"

#include "app_config.h"
#include "chassis_motion.h"
#include "main.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "ui_manager.h"

#define INPUT_FLAG_KEY0 (1UL << 0)
#define INPUT_FLAG_KEY1 (1UL << 1)
#define INPUT_FLAG_ESTOP (1UL << 2)
#define INPUT_FLAG_LIMIT (1UL << 3)

static osEventFlagsId_t g_events;

static uint8_t Pressed(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET;
}

void InputManager_Init(void)
{
  g_events = osEventFlagsNew(0);
}

void InputManager_HandleExti(uint16_t gpio_pin)
{
  uint32_t flag = 0U;
  if (gpio_pin == KEY_RUN_STOP_Pin) flag = INPUT_FLAG_KEY0;
  else if (gpio_pin == KEY_DIR_TOGGLE_Pin) flag = INPUT_FLAG_KEY1;
  else if (gpio_pin == ESTOP_IN_Pin) flag = INPUT_FLAG_ESTOP;
  else if ((gpio_pin == LIMIT_X_MIN_Pin) || (gpio_pin == LIMIT_X_MAX_Pin) ||
           (gpio_pin == LIMIT_Z_MAX_Pin)
#ifdef LIMIT_Z_MIN_Pin
           || (gpio_pin == LIMIT_Z_MIN_Pin)
#endif
           ) flag = INPUT_FLAG_LIMIT;
  if ((flag != 0U) && (g_events != NULL)) (void)osEventFlagsSet(g_events, flag);
}

void InputManager_Task(void)
{
  AppConfig config;
  for (;;)
  {
    uint32_t flags = osEventFlagsWait(g_events, INPUT_FLAG_KEY0 | INPUT_FLAG_KEY1 |
                                      INPUT_FLAG_ESTOP | INPUT_FLAG_LIMIT,
                                      osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U) continue;
    AppConfig_GetSnapshot(&config);
    if ((flags & INPUT_FLAG_LIMIT) != 0U) continue;
    if ((flags & INPUT_FLAG_ESTOP) != 0U)
    {
      SafetyManager_TriggerEstop();
      continue;
    }
    if ((flags & INPUT_FLAG_KEY0) != 0U)
    {
      osDelay(config.key_debounce_ms);
      if (Pressed(KEY_RUN_STOP_GPIO_Port, KEY_RUN_STOP_Pin))
      {
        /* 当前实机调试：K0 按标准角度在 0、90、180 度之间往返。 */
        ServoControl_StepAnglePingPong(0U, 90U);
        while (Pressed(KEY_RUN_STOP_GPIO_Port, KEY_RUN_STOP_Pin)) osDelay(10U);
      }
    }
    if ((flags & INPUT_FLAG_KEY1) != 0U)
    {
      uint32_t pressed_ms = 0U;
      osDelay(config.key_debounce_ms);
      while (Pressed(KEY_DIR_TOGGLE_GPIO_Port, KEY_DIR_TOGGLE_Pin))
      {
        osDelay(10U);
        pressed_ms += 10U;
      }
      if (pressed_ms >= config.key_long_press_ms) UiManager_NextPage();
      else if (UiManager_GetPage() == UI_PAGE_MOTOR_TUNING) ChassisMotion_AdjustPidTarget(-10);
      else StepperAxis_ToggleDirection(STEPPER_AXIS_Z);
    }
  }
}
