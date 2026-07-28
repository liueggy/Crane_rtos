#include "photo_sensor.h"

#include "main.h"

static volatile uint8_t g_state[PHOTO_SENSOR_COUNT];
static volatile uint8_t g_changed_mask;

static uint8_t PhotoSensor_Read(uint8_t index)
{
  static GPIO_TypeDef *const ports[PHOTO_SENSOR_COUNT] = {
    PHOTO_SENSOR_1_GPIO_Port, PHOTO_SENSOR_2_GPIO_Port, PHOTO_SENSOR_3_GPIO_Port
  };
  static const uint16_t pins[PHOTO_SENSOR_COUNT] = {
    PHOTO_SENSOR_1_Pin, PHOTO_SENSOR_2_Pin, PHOTO_SENSOR_3_Pin
  };

  if (index >= PHOTO_SENSOR_COUNT) return 0U;
  return (HAL_GPIO_ReadPin(ports[index], pins[index]) == GPIO_PIN_SET) ? 1U : 0U;
}

void PhotoSensor_Init(void)
{
  for (uint8_t index = 0U; index < PHOTO_SENSOR_COUNT; ++index)
  {
    g_state[index] = PhotoSensor_Read(index);
  }
  g_changed_mask = 0U;
}

void PhotoSensor_HandleExti(uint16_t gpio_pin)
{
  uint8_t index;

  if (gpio_pin == PHOTO_SENSOR_1_Pin) index = 0U;
  else if (gpio_pin == PHOTO_SENSOR_2_Pin) index = 1U;
  else if (gpio_pin == PHOTO_SENSOR_3_Pin) index = 2U;
  else return;

  g_state[index] = PhotoSensor_Read(index);
  g_changed_mask |= (uint8_t)(1U << index);
}

uint8_t PhotoSensor_GetState(uint8_t index)
{
  uint8_t state;
  if (index >= PHOTO_SENSOR_COUNT) return 0U;

  /* 控制任务直接复核GPIO，避免一次EXTI边沿丢失后缓存状态长期错误。 */
  state = PhotoSensor_Read(index);
  g_state[index] = state;
  return state;
}

uint8_t PhotoSensor_ConsumeChangedMask(void)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t mask;

  __disable_irq();
  mask = g_changed_mask;
  g_changed_mask = 0U;
  __set_PRIMASK(primask);
  return mask;
}
