#include "chassis_motion.h"

#include "app_config.h"
#include "app_state.h"
#include "motor_control.h"

#define CHASSIS_OPEN_LOOP_PWM 75
#define CHASSIS_DEFAULT_RPM 110
#define CHASSIS_MIN_RPM 20

static volatile uint8_t g_manual_active;
static volatile uint8_t g_running;
static volatile uint8_t g_reverse;
static volatile uint8_t g_closed_loop;
static volatile int16_t g_target_rpm;
static int16_t g_open_loop_command;
static uint8_t g_applied_closed_loop;
static uint8_t g_route_active;

void ChassisMotion_Init(void)
{
  g_manual_active = 0U;
  g_running = 0U;
  g_reverse = 0U;
  g_closed_loop = 1U;
  g_target_rpm = CHASSIS_DEFAULT_RPM;
  g_open_loop_command = 0;
  g_applied_closed_loop = 1U;
  g_route_active = 0U;
}

void ChassisMotion_ToggleRunning(void)
{
  ChassisMotion_SetRunning(g_running ? 0U : 1U);
}

void ChassisMotion_SetRunning(uint8_t running)
{
  g_running = running ? 1U : 0U;
  g_manual_active = 1U;
  AppState_SetRunEnabled(g_running);
}

void ChassisMotion_StopManual(void)
{
  ChassisMotion_SetRunning(0U);
}

uint8_t ChassisMotion_IsRunning(void)
{
  return g_running;
}

uint8_t ChassisMotion_SetDirection(uint8_t reverse)
{
  AppState state;
  AppState_GetSnapshot(&state);
  if (g_running) return 0U;
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    if (state.pwm_command[i] != 0) return 0U;
  }
  g_reverse = reverse ? 1U : 0U;
  g_manual_active = 1U;
  return 1U;
}

uint8_t ChassisMotion_IsDirectionReverse(void)
{
  return g_reverse;
}

uint8_t ChassisMotion_ToggleClosedLoop(void)
{
  return ChassisMotion_SetClosedLoop(g_closed_loop ? 0U : 1U);
}

uint8_t ChassisMotion_SetClosedLoop(uint8_t enabled)
{
  AppState state;
  AppState_GetSnapshot(&state);
  if (g_running) return 0U;
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    if (state.pwm_command[i] != 0) return 0U;
  }
  g_closed_loop = enabled ? 1U : 0U;
  g_manual_active = 1U;
  return 1U;
}

uint8_t ChassisMotion_IsClosedLoop(void)
{
  return g_closed_loop;
}

void ChassisMotion_AdjustTargetRpm(int16_t delta_rpm)
{
  ChassisMotion_SetTargetRpm((int16_t)(g_target_rpm + delta_rpm));
}

void ChassisMotion_SetTargetRpm(int16_t target_rpm)
{
  AppConfig config;
  int32_t target = target_rpm;
  AppConfig_GetSnapshot(&config);
  if (target < CHASSIS_MIN_RPM) target = CHASSIS_MIN_RPM;
  if (target > (int32_t)config.maximum_rpm) target = (int32_t)config.maximum_rpm;
  g_target_rpm = (int16_t)target;
}

int16_t ChassisMotion_GetTargetRpm(void)
{
  return g_target_rpm;
}

void ChassisMotion_Stop(void)
{
  g_manual_active = 0U;
  g_running = 0U;
  g_open_loop_command = 0;
  g_route_active = 0U;
  MotorControl_Reset();
  AppState_SetRunEnabled(0U);
}

uint8_t ChassisMotion_StartRouteSegment(int16_t distance_mm, int16_t turn_deg,
                                        uint16_t speed_rpm, uint16_t timeout_ms)
{
  (void)distance_mm;
  (void)turn_deg;
  (void)timeout_ms;
  if (speed_rpm == 0U) return 0U;
  MotorControl_SetAllTargetRpm((float)speed_rpm);
  g_route_active = 1U;
  return 1U;
}

uint8_t ChassisMotion_IsRouteSegmentDone(void)
{
  /* 地标/距离闭环接入前，不在底盘层虚报到位。 */
  return g_route_active ? 0U : 1U;
}

void ChassisMotion_TaskStep(void)
{
  AppState state;
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

  if (!g_manual_active)
  {
    MotorControl_Update(0U);
    return;
  }

  if (g_closed_loop)
  {
    float target = g_reverse ? -(float)g_target_rpm : (float)g_target_rpm;
    if (!g_applied_closed_loop)
    {
      MotorControl_Reset();
      g_applied_closed_loop = 1U;
    }
    MotorControl_SetAllTargetRpm(target);
    MotorControl_Update(g_running);
    return;
  }

  if (g_applied_closed_loop)
  {
    MotorControl_Reset();
    g_open_loop_command = 0;
    g_applied_closed_loop = 0U;
  }
  if (!g_running)
    g_open_loop_command = 0;
  else
    g_open_loop_command = g_reverse ? -CHASSIS_OPEN_LOOP_PWM : CHASSIS_OPEN_LOOP_PWM;

  int16_t commands[APP_MOTOR_COUNT];
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i) commands[i] = g_open_loop_command;
  MotorControl_UpdateOpenLoop(commands);
}
