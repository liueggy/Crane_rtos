#ifndef STEPPER_AXIS_H
#define STEPPER_AXIS_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum
{
  STEPPER_AXIS_X = 0,
  STEPPER_AXIS_Z,
  STEPPER_AXIS_COUNT
} StepperAxisId;

void StepperAxis_Init(TIM_HandleTypeDef *timer);
void StepperAxis_SetEnabled(StepperAxisId axis, uint8_t enabled);
void StepperAxis_SetDirectionReverse(StepperAxisId axis, uint8_t reverse);
void StepperAxis_ToggleDirection(StepperAxisId axis);
void StepperAxis_StopAll(void);
void StepperAxis_UpdateTelemetry(void);

#endif
