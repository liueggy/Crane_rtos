#include "encoder.h"

#include "app_config.h"
#include "main.h"
#include "cmsis_gcc.h"

typedef struct
{
  GPIO_TypeDef *a_port;
  uint16_t a_pin;
  GPIO_TypeDef *b_port;
  uint16_t b_pin;
} EncoderHardware;

/* 每个编码器使用 A 相 EXTI，B 相作为方向判定输入。 */
static const EncoderHardware k_encoder_hardware[APP_MOTOR_COUNT] = {
  {ENC_M1_A_GPIO_Port, ENC_M1_A_Pin, ENC_M1_B_GPIO_Port, ENC_M1_B_Pin},
  {ENC_M2_A_GPIO_Port, ENC_M2_A_Pin, ENC_M2_B_GPIO_Port, ENC_M2_B_Pin},
  {ENC_M3_A_GPIO_Port, ENC_M3_A_Pin, ENC_M3_B_GPIO_Port, ENC_M3_B_Pin},
  {ENC_M4_A_GPIO_Port, ENC_M4_A_Pin, ENC_M4_B_GPIO_Port, ENC_M4_B_Pin},
};

static volatile int32_t g_encoder_count[APP_MOTOR_COUNT];
static int32_t g_encoder_last_count[APP_MOTOR_COUNT];
static int8_t g_encoder_polarity[APP_MOTOR_COUNT];

void Encoder_Init(void)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    g_encoder_count[i] = 0;
    g_encoder_last_count[i] = 0;
    g_encoder_polarity[i] = config.encoder_polarity[i];
  }
}

void Encoder_HandleExti(uint16_t gpio_pin)
{
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    if (gpio_pin == k_encoder_hardware[i].a_pin)
    {
      /* 正交编码器 A 相边沿到来时读取 B 相即可获得方向。 */
      int32_t direction = (HAL_GPIO_ReadPin(k_encoder_hardware[i].b_port,
                                            k_encoder_hardware[i].b_pin) == GPIO_PIN_SET) ? 1 : -1;
      g_encoder_count[i] += direction * g_encoder_polarity[i];
      break;
    }
  }
}

int32_t Encoder_GetCount(uint8_t index)
{
  int32_t count = 0;
  uint32_t primask;
  if (index >= APP_MOTOR_COUNT)
  {
    return 0;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  count = g_encoder_count[index];
  __set_PRIMASK(primask);
  return count;
}

int32_t Encoder_GetDelta(uint8_t index)
{
  int32_t count;
  int32_t delta;
  uint32_t primask;
  if (index >= APP_MOTOR_COUNT)
  {
    return 0;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  count = g_encoder_count[index];
  __set_PRIMASK(primask);
  /* 先取快照再计算增量，供固定周期的速度环使用。 */
  delta = count - g_encoder_last_count[index];
  g_encoder_last_count[index] = count;
  return delta;
}
