#ifndef INFRARED_REMOTE_H
#define INFRARED_REMOTE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 示例遥控器采用的标准 NEC 命令码。 */
#define IR_REMOTE_CMD_POWER 0x45U
#define IR_REMOTE_CMD_UP    0x46U
#define IR_REMOTE_CMD_DOWN  0x15U

/* 初始化 TIM4_CH4 红外接收；TIM4 计数频率必须为 1 MHz。 */
HAL_StatusTypeDef InfraredRemote_Init(TIM_HandleTypeDef *htim);

/* 在 HAL_TIM_IC_CaptureCallback 中转发 TIM4 的捕获事件。 */
void InfraredRemote_HandleCapture(TIM_HandleTypeDef *htim);

/* 在普通任务上下文中执行已解码的遥控命令。 */
void InfraredRemote_Process(void);

uint8_t InfraredRemote_GetLastCommand(void);

#endif
