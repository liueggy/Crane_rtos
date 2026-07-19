#include "buzzer.h"

#include "main.h"

static uint8_t g_active;
static uint32_t g_stop_tick;

void Buzzer_Init(void)
{
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
  g_active = 0U;
  g_stop_tick = 0U;
}

void Buzzer_Beep(uint32_t duration_ms)
{
  if (duration_ms == 0U) return;
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
  g_active = 1U;
  g_stop_tick = HAL_GetTick() + duration_ms;
}

void Buzzer_Process(void)
{
  if ((g_active != 0U) && ((int32_t)(HAL_GetTick() - g_stop_tick) >= 0))
  {
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    g_active = 0U;
  }
}
