#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

void MotorControl_Init(void);
/* 设置单轮目标转速，单位 RPM；实际输出会经过加速斜坡限制。 */
void MotorControl_SetTargetRpm(uint8_t index, float rpm);
void MotorControl_SetAllTargetRpm(float rpm);
void MotorControl_Update(uint8_t enabled);
/* 清零积分和目标，并关闭四路电机。 */
void MotorControl_Reset(void);

#endif
