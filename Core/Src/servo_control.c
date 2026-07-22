#include "servo_control.h"

#include "app_config.h"

static TIM_HandleTypeDef *g_timer;
static uint16_t g_pulse_us[2] = {1000U, 1000U};
static uint16_t g_angle[2];
static int8_t g_step_direction[2] = {1, 1};

static uint16_t ClampPulse(uint8_t index, uint16_t pulse_us)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);
  if (index >= 2U) return 0U;
  if (pulse_us < config.servo_min_us[index]) return config.servo_min_us[index];
  if (pulse_us > config.servo_max_us[index]) return config.servo_max_us[index];
  return pulse_us;
}

void ServoControl_Init(TIM_HandleTypeDef *timer)
{
  g_timer = timer;
}

void ServoControl_SetPulseUs(uint8_t index, uint16_t pulse_us)
{
  uint32_t channel;
  if ((g_timer == NULL) || (index >= 2U)) return;
  channel = (index == 0U) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
  g_pulse_us[index] = ClampPulse(index, pulse_us);
  __HAL_TIM_SET_COMPARE(g_timer, channel, g_pulse_us[index]);
}

void ServoControl_SetAngle(uint8_t index, uint16_t angle)
{
  AppConfig config;
  uint32_t pulse;
  if (index >= 2U) return;
  AppConfig_GetSnapshot(&config);
  if (config.servo_travel_degrees[index] == 0U) return;
  if (angle > config.servo_travel_degrees[index]) angle = config.servo_travel_degrees[index];
  pulse = config.servo_min_us[index] +
          ((uint32_t)(config.servo_max_us[index] - config.servo_min_us[index]) * angle) /
          config.servo_travel_degrees[index];
  ServoControl_SetPulseUs(index, (uint16_t)pulse);
  g_angle[index] = angle;
}

void ServoControl_AdjustAngle(uint8_t index, int16_t delta_degrees)
{
  AppConfig config;
  int32_t target;
  if (index >= 2U) return;
  AppConfig_GetSnapshot(&config);
  target = (int32_t)g_angle[index] + delta_degrees;
  if (target < 0) target = 0;
  if (target > (int32_t)config.servo_travel_degrees[index])
    target = (int32_t)config.servo_travel_degrees[index];
  ServoControl_SetAngle(index, (uint16_t)target);
}

void ServoControl_StepAnglePingPong(uint8_t index, uint16_t step_degrees)
{
  AppConfig config;
  uint16_t target;

  if ((index >= 2U) || (step_degrees == 0U)) return;
  AppConfig_GetSnapshot(&config);
  if (config.servo_travel_degrees[index] == 0U) return;

  if (g_step_direction[index] > 0)
  {
    if (g_angle[index] + step_degrees >= config.servo_travel_degrees[index])
    {
      target = config.servo_travel_degrees[index];
      g_step_direction[index] = -1;
    }
    else target = g_angle[index] + step_degrees;
  }
  else
  {
    if (g_angle[index] <= step_degrees)
    {
      target = 0U;
      g_step_direction[index] = 1;
    }
    else target = g_angle[index] - step_degrees;
  }

  ServoControl_SetAngle(index, target);
}

uint16_t ServoControl_GetPulseUs(uint8_t index)
{
  return (index < 2U) ? g_pulse_us[index] : 0U;
}

uint16_t ServoControl_GetAngle(uint8_t index)
{
  return (index < 2U) ? g_angle[index] : 0U;
}
