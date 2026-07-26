#ifndef CAMERA_TILT_H
#define CAMERA_TILT_H

#include "stm32f1xx_hal.h"

#include <stdint.h>

#define CAMERA_TILT_LEVEL_PULSE_US 1700U
#define CAMERA_TILT_DOWN_PULSE_US  2500U

void CameraTilt_Init(TIM_HandleTypeDef *timer);
void CameraTilt_SetPulseUs(uint16_t pulse_us);
void CameraTilt_SetLevel(void);
void CameraTilt_SetDown(void);
uint16_t CameraTilt_GetPulseUs(void);

#endif
