#include "camera_tilt.h"

static TIM_HandleTypeDef *g_timer;
static uint16_t g_pulse_us = CAMERA_TILT_LEVEL_PULSE_US;

void CameraTilt_Init(TIM_HandleTypeDef *timer)
{
  g_timer = timer;
  CameraTilt_SetLevel();
}

void CameraTilt_SetPulseUs(uint16_t pulse_us)
{
  if (g_timer == NULL) return;
  if (pulse_us < CAMERA_TILT_LEVEL_PULSE_US) pulse_us = CAMERA_TILT_LEVEL_PULSE_US;
  if (pulse_us > CAMERA_TILT_DOWN_PULSE_US) pulse_us = CAMERA_TILT_DOWN_PULSE_US;
  g_pulse_us = pulse_us;
  __HAL_TIM_SET_COMPARE(g_timer, TIM_CHANNEL_3, g_pulse_us);
}

void CameraTilt_SetLevel(void)
{
  CameraTilt_SetPulseUs(CAMERA_TILT_LEVEL_PULSE_US);
}

void CameraTilt_SetDown(void)
{
  CameraTilt_SetPulseUs(CAMERA_TILT_DOWN_PULSE_US);
}

uint16_t CameraTilt_GetPulseUs(void)
{
  return g_pulse_us;
}
