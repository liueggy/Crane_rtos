#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void ServoControl_Init(TIM_HandleTypeDef *timer);
void ServoControl_SetPulseUs(uint8_t index, uint16_t pulse_us);
void ServoControl_SetAngle(uint8_t index, uint16_t angle);
uint8_t ServoControl_StartSlew(uint8_t index, uint16_t target_angle,
                               uint16_t degrees_per_second);
void ServoControl_Process(void);
void ServoControl_CancelSlew(uint8_t index);
uint8_t ServoControl_IsSlewActive(uint8_t index);
uint16_t ServoControl_GetSlewTarget(uint8_t index);
void ServoControl_ResetToInitial(uint8_t index);
void ServoControl_AdjustAngle(uint8_t index, int16_t delta_degrees);
void ServoControl_StepAnglePingPong(uint8_t index, uint16_t step_degrees);
uint16_t ServoControl_GetPulseUs(uint8_t index);
uint16_t ServoControl_GetAngle(uint8_t index);

#endif
