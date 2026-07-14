#include "chassis_motion.h"

#include "app_config.h"
#include "app_state.h"
#include "dc_motor.h"
#include "motor_control.h"
#include "ui_manager.h"
#include <stdio.h>

static volatile uint16_t g_test_pwm;
static volatile uint8_t g_test_reverse;
static volatile int16_t g_pid_target_rpm;
static int16_t g_applied_command;
static uint8_t g_pid_mode;
static uint8_t g_route_active;
static uint32_t g_route_started_ms;
static uint16_t g_route_timeout_ms;

void ChassisMotion_Init(void)
{
  g_test_pwm = 0U;
  g_test_reverse = 0U;
  g_pid_target_rpm = 0;
  g_applied_command = 0;
  g_pid_mode = 0U;
  g_route_active = 0U;
  AppState_SetDcTestState(0U, 0U, 0U);
}

void ChassisMotion_SelectNextTestGear(void)
{
  AppConfig config;
  AppState state;
  AppConfig_GetSnapshot(&config);
  AppState_GetSnapshot(&state);
  if (state.estop_active) return;
  g_test_pwm = ((g_test_pwm + config.motor_test_pwm_step) > config.motor_test_pwm_limit) ?
               0U : (uint16_t)(g_test_pwm + config.motor_test_pwm_step);
  AppState_SetRunEnabled(g_test_pwm > 0U);
  AppState_SetDcTestState((uint8_t)((g_test_pwm + config.motor_test_pwm_step - 1U) /
                                    config.motor_test_pwm_step),
                          g_test_reverse, g_test_pwm);
}

void ChassisMotion_ToggleTestDirection(void)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);
  g_test_reverse ^= 1U;
  AppState_SetDcTestState((uint8_t)((g_test_pwm + config.motor_test_pwm_step - 1U) /
                                    config.motor_test_pwm_step),
                          g_test_reverse, g_test_pwm);
}

void ChassisMotion_AdjustPidTarget(int16_t delta_rpm)
{
  AppConfig config;
  int32_t target;
  AppConfig_GetSnapshot(&config);
  target = (int32_t)g_pid_target_rpm + delta_rpm;
  if (target > (int32_t)config.maximum_rpm) target = (int32_t)config.maximum_rpm;
  if (target < -(int32_t)config.maximum_rpm) target = -(int32_t)config.maximum_rpm;
  g_pid_target_rpm = (int16_t)target;
}

void ChassisMotion_Stop(void)
{
  g_test_pwm = 0U;
  g_pid_target_rpm = 0;
  g_applied_command = 0;
  g_route_active = 0U;
  MotorControl_Reset();
  AppState_SetRunEnabled(0U);
  AppState_SetDcTestState(0U, g_test_reverse, 0U);
}

uint8_t ChassisMotion_StartRouteSegment(int16_t distance_mm, int16_t turn_deg,
                                        uint16_t speed_rpm, uint16_t timeout_ms)
{
  (void)distance_mm;
  (void)turn_deg;
  /* 先让路线任务使用四轮同步速度接口；距离/地标闭环会在底盘标定后接入。 */
  if (speed_rpm == 0U) return 0U;
  MotorControl_SetAllTargetRpm((float)speed_rpm);
  g_route_active = 1U;
  g_route_started_ms = HAL_GetTick();
  g_route_timeout_ms = timeout_ms;
  return 1U;
}

uint8_t ChassisMotion_IsRouteSegmentDone(void)
{
  if (!g_route_active) return 1U;
  /* 未接入地标前绝不自动宣告到位，超时会由上层状态机转入故障。 */
  return 0U;
}

void ChassisMotion_TaskStep(void)
{
  AppConfig config;
  AppState state;
  int16_t requested;
  uint8_t tuning_page;
  AppConfig_GetSnapshot(&config);
  AppState_GetSnapshot(&state);
  if (state.estop_active)
  {
    ChassisMotion_Stop();
    return;
  }
  if (g_route_active)
  {
    MotorControl_Update(1U);
    return;
  }
  tuning_page = (UiManager_GetPage() == UI_PAGE_MOTOR_TUNING) ? 1U : 0U;
  if (tuning_page != g_pid_mode)
  {
    MotorControl_Reset();
    g_applied_command = 0;
    g_pid_mode = tuning_page;
  }
  if (g_pid_mode)
  {
    MotorControl_UpdatePidSingle(0U, (float)g_pid_target_rpm);
    return;
  }
  requested = g_test_reverse ? -(int16_t)g_test_pwm : (int16_t)g_test_pwm;
  if (((g_applied_command > 0) && (requested < 0)) ||
      ((g_applied_command < 0) && (requested > 0))) requested = 0;
  if (g_applied_command < requested) ++g_applied_command;
  else if (g_applied_command > requested) --g_applied_command;
  MotorControl_UpdateOpenLoopSingle(0U, g_applied_command);
}
