#include "dc_motor.h"

#include "app_config.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t forward_channel;
  uint32_t reverse_channel;
} DcMotorHardware;

/* 将逻辑电机编号映射到 L298N 方向引脚和 TIM3 PWM 通道。 */
static const DcMotorHardware k_motor_hardware[APP_MOTOR_COUNT] = {
  {&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2},
  {&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4},
  {&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2},
  {&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4},
};

static int8_t g_motor_polarity[APP_MOTOR_COUNT];
static uint16_t g_pwm_max;

void DcMotor_Init(TIM_HandleTypeDef *tim3, TIM_HandleTypeDef *tim8)
{
  AppConfig config;
  (void)tim3;
  (void)tim8;
  AppConfig_GetSnapshot(&config);
  g_pwm_max = config.pwm_max;
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    g_motor_polarity[i] = config.motor_polarity[i];
  }
  DcMotor_StopAll();
}

void DcMotor_SetCommand(uint8_t index, int16_t command)
{
  uint16_t pwm;
  if (index >= APP_MOTOR_COUNT)
  {
    return;
  }
  /* 电机极性用于适配四个安装方向不同的轮子。 */
  command *= g_motor_polarity[index];
  if (command > (int16_t)g_pwm_max)
  {
    command = (int16_t)g_pwm_max;
  }
  else if (command < -(int16_t)g_pwm_max)
  {
    command = -(int16_t)g_pwm_max;
  }

  /* 每个电机使用IN1/IN2双PWM：正转PWM在IN1，反转PWM在IN2。 */
  if (command > 0)
  {
    pwm = (uint16_t)command;
    __HAL_TIM_SET_COMPARE(k_motor_hardware[index].timer,
                          k_motor_hardware[index].forward_channel, pwm);
    __HAL_TIM_SET_COMPARE(k_motor_hardware[index].timer,
                          k_motor_hardware[index].reverse_channel, 0U);
  }
  else if (command < 0)
  {
    pwm = (uint16_t)(-command);
    __HAL_TIM_SET_COMPARE(k_motor_hardware[index].timer,
                          k_motor_hardware[index].forward_channel, 0U);
    __HAL_TIM_SET_COMPARE(k_motor_hardware[index].timer,
                          k_motor_hardware[index].reverse_channel, pwm);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(k_motor_hardware[index].timer,
                          k_motor_hardware[index].forward_channel, 0U);
    __HAL_TIM_SET_COMPARE(k_motor_hardware[index].timer,
                          k_motor_hardware[index].reverse_channel, 0U);
  }
}

void DcMotor_StopAll(void)
{
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    DcMotor_SetCommand(i, 0);
  }
}
