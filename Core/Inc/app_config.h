#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#define APP_MOTOR_COUNT 4U /* 四个轮毂电机共用同一套控制参数模型。 */

/* 运行期可替换的机器人参数快照。 */
typedef struct
{
  float speed_kp[APP_MOTOR_COUNT];
  float speed_ki[APP_MOTOR_COUNT];
  float speed_feedforward[APP_MOTOR_COUNT];
  float speed_sync_kp;
  float acceleration_rpm_s;
  float deceleration_rpm_s;
  float encoder_counts_per_output_rev;
  float speed_filter_alpha;
  float maximum_rpm;
  uint16_t pwm_max;
  uint16_t pwm_deadband;
  uint16_t motor_test_pwm_step;
  uint16_t motor_test_pwm_limit;
  uint16_t motor_control_period_ms;
  uint16_t ui_refresh_period_ms;
  uint16_t key_debounce_ms;
  uint16_t key_long_press_ms;
  uint16_t servo_min_us[2];
  uint16_t servo_max_us[2];
  uint16_t servo_travel_degrees[2];
  uint16_t gripper_closed_degrees;
  uint16_t gripper_open_degrees;
  uint16_t gripper_release_degrees;
  int8_t motor_polarity[APP_MOTOR_COUNT];
  int8_t encoder_polarity[APP_MOTOR_COUNT];
} AppConfig;

void AppConfig_Init(void);
/* 获取一致的参数副本，供控制任务和显示任务读取。 */
void AppConfig_GetSnapshot(AppConfig *snapshot);
/* 校验并原子替换当前参数；返回 1 表示成功。 */
uint8_t AppConfig_Replace(const AppConfig *candidate);
/* 恢复编译期默认参数。 */
void AppConfig_LoadDefaults(void);

#endif
