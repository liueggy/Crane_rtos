#include "dc_motor.h"

#include "app_config.h"
#include "main.h"

typedef struct
{
  GPIO_TypeDef *in1_port;
  uint16_t in1_pin;
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;
  uint32_t pwm_channel;
} DcMotorHardware;

/* 将逻辑电机编号映射到 L298N 方向引脚和 TIM3 PWM 通道。 */
static const DcMotorHardware k_motor_hardware[APP_MOTOR_COUNT] = {
  {DC_M1_IN1_GPIO_Port, DC_M1_IN1_Pin, DC_M1_IN2_GPIO_Port, DC_M1_IN2_Pin, TIM_CHANNEL_1},
  {DC_M2_IN1_GPIO_Port, DC_M2_IN1_Pin, DC_M2_IN2_GPIO_Port, DC_M2_IN2_Pin, TIM_CHANNEL_2},
  {DC_M3_IN1_GPIO_Port, DC_M3_IN1_Pin, DC_M3_IN2_GPIO_Port, DC_M3_IN2_Pin, TIM_CHANNEL_3},
  {DC_M4_IN1_GPIO_Port, DC_M4_IN1_Pin, DC_M4_IN2_GPIO_Port, DC_M4_IN2_Pin, TIM_CHANNEL_4},
};

static TIM_HandleTypeDef *g_pwm_timer;
static int8_t g_motor_polarity[APP_MOTOR_COUNT];
static uint16_t g_pwm_max;

void DcMotor_Init(TIM_HandleTypeDef *pwm_timer)
{
  AppConfig config;
  g_pwm_timer = pwm_timer;
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
  if ((index >= APP_MOTOR_COUNT) || (g_pwm_timer == NULL))
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

  /* L298N 使用两根方向线决定正反转，PWM 决定驱动力大小。 */
  if (command > 0)
  {
    HAL_GPIO_WritePin(k_motor_hardware[index].in1_port, k_motor_hardware[index].in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(k_motor_hardware[index].in2_port, k_motor_hardware[index].in2_pin, GPIO_PIN_RESET);
    pwm = (uint16_t)command;
  }
  else if (command < 0)
  {
    HAL_GPIO_WritePin(k_motor_hardware[index].in1_port, k_motor_hardware[index].in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(k_motor_hardware[index].in2_port, k_motor_hardware[index].in2_pin, GPIO_PIN_SET);
    pwm = (uint16_t)(-command);
  }
  else
  {
    HAL_GPIO_WritePin(k_motor_hardware[index].in1_port, k_motor_hardware[index].in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(k_motor_hardware[index].in2_port, k_motor_hardware[index].in2_pin, GPIO_PIN_RESET);
    pwm = 0U;
  }
  __HAL_TIM_SET_COMPARE(g_pwm_timer, k_motor_hardware[index].pwm_channel, pwm);
}

void DcMotor_StopAll(void)
{
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    DcMotor_SetCommand(i, 0);
  }
}
