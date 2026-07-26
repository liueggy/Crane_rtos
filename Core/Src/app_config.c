#include "app_config.h"

#include "cmsis_gcc.h"
#include <string.h>

/* 初始调试参数集中放在这里，后续可由串口或外接面板替换。 */
static const AppConfig k_default_config = {
  .speed_kp = {0.80f, 0.80f, 0.80f, 0.80f},
  .speed_ki = {2.50f, 2.50f, 2.50f, 2.50f},
  .speed_feedforward = {0.65f, 0.65f, 0.65f, 0.65f},
  .speed_sync_kp = 0.20f,
  .acceleration_rpm_s = 80.0f,
  .deceleration_rpm_s = 120.0f,
  .encoder_counts_per_output_rev = 374.0f,
  .speed_filter_alpha = 0.25f,
  .maximum_rpm = 130.0f,
  .pwm_max = 99U,
  .pwm_deadband = 8U,
  .motor_control_period_ms = 10U,
  /* 50ms累计测速把单脉冲分辨率由约16RPM改善到约3.2RPM。 */
  .speed_measurement_period_ms = 50U,
  .ui_refresh_period_ms = 100U,
  .key_debounce_ms = 20U,
  .servo_min_us = {500U, 1000U},
  .servo_max_us = {2500U, 2000U},
  .servo_travel_degrees = {270U, 180U},
  /* 旋转舵机规定0度对应实机6度；夹爪角度直接使用实机标定值。 */
  .servo_angle_offset_degrees = {6U, 0U},
  .servo_command_min_degrees = {0U, 10U},
  .servo_command_max_degrees = {264U, 100U},
  .servo_initial_degrees = {0U, 10U},
  .gripper_closed_degrees = 10U,
  .gripper_open_degrees = 100U,
  .gripper_release_degrees = 45U,
  /* 整车规定正方向：M1/M2安装方向相反，需在驱动层反相。 */
  .motor_polarity = {-1, -1, 1, 1},
  /* 规定正向下M1/M2原始计数递减，反馈层反相后四路统一为正。 */
  .encoder_polarity = {-1, -1, 1, 1},
};

/* 控制任务和 UI 任务共享的当前参数。 */
static AppConfig g_config;

static uint8_t AppConfig_IsValid(const AppConfig *config)
{
  /* 先做基础范围检查，避免错误参数直接进入闭环计算。 */
  if ((config == NULL) ||
      (config->encoder_counts_per_output_rev < 1.0f) ||
      (config->maximum_rpm <= 0.0f) ||
      (config->pwm_max == 0U) || (config->pwm_max > 99U) ||
      (config->motor_control_period_ms < 5U) ||
      (config->speed_measurement_period_ms < config->motor_control_period_ms) ||
      (config->speed_measurement_period_ms > 500U) ||
      (config->speed_filter_alpha < 0.0f) ||
      (config->speed_filter_alpha > 1.0f))
  {
    return 0U;
  }
  for (uint8_t i = 0U; i < 2U; ++i)
  {
    if ((config->servo_min_us[i] >= config->servo_max_us[i]) ||
        (config->servo_travel_degrees[i] == 0U) ||
        (config->servo_angle_offset_degrees[i] >= config->servo_travel_degrees[i]) ||
        (config->servo_command_min_degrees[i] > config->servo_command_max_degrees[i]) ||
        (config->servo_command_max_degrees[i] >
         (config->servo_travel_degrees[i] - config->servo_angle_offset_degrees[i])) ||
        (config->servo_initial_degrees[i] < config->servo_command_min_degrees[i]) ||
        (config->servo_initial_degrees[i] > config->servo_command_max_degrees[i]))
    {
      return 0U;
    }
  }
  if ((config->gripper_closed_degrees > config->servo_travel_degrees[1]) ||
      (config->gripper_open_degrees > config->servo_travel_degrees[1]) ||
      (config->gripper_release_degrees > config->servo_travel_degrees[1]))
  {
    return 0U;
  }
  return 1U;
}

void AppConfig_Init(void)
{
  AppConfig_LoadDefaults();
}

void AppConfig_LoadDefaults(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  /* 复制期间暂时关中断，避免读取快照时出现半更新结构体。 */
  g_config = k_default_config;
  __set_PRIMASK(primask);
}

void AppConfig_GetSnapshot(AppConfig *snapshot)
{
  uint32_t primask;
  if (snapshot == NULL)
  {
    return;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  /* 调用者只使用副本，不直接访问共享配置。 */
  memcpy(snapshot, &g_config, sizeof(*snapshot));
  __set_PRIMASK(primask);
}

uint8_t AppConfig_Replace(const AppConfig *candidate)
{
  uint32_t primask;
  if (!AppConfig_IsValid(candidate))
  {
    return 0U;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  memcpy(&g_config, candidate, sizeof(g_config));
  __set_PRIMASK(primask);
  return 1U;
}
