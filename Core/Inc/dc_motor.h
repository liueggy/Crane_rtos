#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 初始化四路 L298N PWM 和方向输出。 */
void DcMotor_Init(TIM_HandleTypeDef *pwm_timer);
/* command 范围为 -PWM_MAX 到 +PWM_MAX，正负号表示方向。 */
void DcMotor_SetCommand(uint8_t index, int16_t command);
/* 发生急停、故障或任务退出时关闭全部电机。 */
void DcMotor_StopAll(void);

#endif
