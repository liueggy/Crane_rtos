#ifndef APP_STATE_H
#define APP_STATE_H

#include "app_config.h"
#include <stdint.h>

typedef enum
{
  APP_MODE_IDLE = 0,
  APP_MODE_MANUAL,
  APP_MODE_AUTO,
  APP_MODE_FAULT
} AppMode;

/* 面向 UI、调试器和上层状态机的整机遥测快照。 */
typedef struct
{
  AppMode mode;
  uint8_t run_enabled;
  uint8_t estop_active;
  uint8_t k230_online;
  uint8_t ui_page;
  uint8_t stepper_enabled;
  uint8_t stepper_direction_reverse;
  uint16_t stepper_pulse;
  uint8_t dc_test_gear;
  uint8_t dc_test_direction_reverse;
  uint16_t dc_test_pwm_target;
  int32_t encoder_count[APP_MOTOR_COUNT];
  float target_rpm[APP_MOTOR_COUNT];
  float measured_rpm[APP_MOTOR_COUNT];
  int16_t pwm_command[APP_MOTOR_COUNT];
  uint32_t fault_flags;
} AppState;

void AppState_Init(void);
/* 获取一致的状态副本，避免显示过程中读取到不完整数据。 */
void AppState_GetSnapshot(AppState *snapshot);
void AppState_SetRunEnabled(uint8_t enabled);
uint8_t AppState_GetRunEnabled(void);
void AppState_SetEstopActive(uint8_t active);
void AppState_SetMode(AppMode mode);
void AppState_SetK230Online(uint8_t online);
void AppState_SetFaultFlags(uint32_t flags);
void AppState_SetUiPage(uint8_t page);
void AppState_SetStepperTelemetry(uint8_t enabled, uint8_t direction_reverse,
                                  uint16_t pulse);
void AppState_SetDcTestState(uint8_t gear, uint8_t direction_reverse,
                             uint16_t pwm_target);
void AppState_SetMotorTelemetry(uint8_t index, int32_t count, float target_rpm,
                                float measured_rpm, int16_t pwm_command);

#endif
