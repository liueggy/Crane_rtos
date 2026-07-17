#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void ServoControl_Init(TIM_HandleTypeDef *timer);
void ServoControl_SetPulseUs(uint8_t index, uint16_t pulse_us);
void ServoControl_SetAngle(uint8_t index, uint16_t angle);
void ServoControl_StepAnglePingPong(uint8_t index, uint16_t step_degrees);
uint16_t ServoControl_GetPulseUs(uint8_t index);
uint16_t ServoControl_GetAngle(uint8_t index);

#endif
