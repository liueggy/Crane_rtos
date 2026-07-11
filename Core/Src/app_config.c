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
  .speed_filter_alpha = 0.35f,
  .maximum_rpm = 130.0f,
  .pwm_max = 99U,
  .pwm_deadband = 8U,
  .motor_control_period_ms = 10U,
  .ui_refresh_period_ms = 100U,
  .key_debounce_ms = 20U,
  .key_long_press_ms = 800U,
  .servo_min_us = {1000U, 1000U},
  .servo_max_us = {2000U, 2000U},
  .motor_polarity = {1, 1, 1, 1},
  .encoder_polarity = {1, 1, 1, 1},
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
      (config->speed_filter_alpha < 0.0f) ||
      (config->speed_filter_alpha > 1.0f))
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
