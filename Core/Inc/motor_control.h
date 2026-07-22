#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

void MotorControl_Init(void);
/* 设置单轮目标转速，单位 RPM；实际输出会经过加速斜坡限制。 */
void MotorControl_SetTargetRpm(uint8_t index, float rpm);
void MotorControl_SetAllTargetRpm(float rpm);
void MotorControl_Update(uint8_t enabled);
/* 单电机开环验证：保留编码器测速，但不执行 PI 控速。 */
void MotorControl_UpdateOpenLoopSingle(uint8_t index, int16_t pwm_command);
/* 四电机独立开环验证；每路命令范围为 -pwm_max~+pwm_max。 */
void MotorControl_UpdateOpenLoop(const int16_t pwm_command[4]);
/* 单电机PI控速调试，目标单位为RPM，正负号表示方向。 */
void MotorControl_UpdatePidSingle(uint8_t index, float target_rpm);
/* 清零积分和目标，并关闭四路电机。 */
void MotorControl_Reset(void);

#endif
