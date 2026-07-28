#ifndef INFRARED_REMOTE_H
#define INFRARED_REMOTE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 示例遥控器采用的标准 NEC 命令码。 */
#define IR_REMOTE_CMD_POWER 0x45U
#define IR_REMOTE_CMD_UP    0x46U
#define IR_REMOTE_CMD_DOWN  0x15U
#define IR_REMOTE_CMD_RIGHT 0x43U
#define IR_REMOTE_CMD_LEFT  0x44U
#define IR_REMOTE_CMD_1     0x16U
#define IR_REMOTE_CMD_2     0x19U
#define IR_REMOTE_CMD_3     0x0DU
#define IR_REMOTE_CMD_4     0x0CU
#define IR_REMOTE_CMD_5     0x18U
#define IR_REMOTE_CMD_6     0x5EU
#define IR_REMOTE_CMD_7     0x08U
#define IR_REMOTE_CMD_8     0x1CU
#define IR_REMOTE_CMD_9     0x5AU
#define IR_REMOTE_CMD_0     0x42U
#define IR_REMOTE_CMD_VOL_MINUS 0x07U
#define IR_REMOTE_CMD_VOL_PLUS  0x09U

#define IR_REMOTE_DC_MOTOR_COUNT 4U

typedef enum
{
  IR_MOTION_STOP = 0,
  IR_MOTION_FORWARD,
  IR_MOTION_REVERSE
} InfraredMotionState;

typedef enum
{
  IR_CONTROL_AXIS_X = 0,
  IR_CONTROL_AXIS_Z,
  IR_CONTROL_AXIS_COUNT
} InfraredControlAxis;

typedef enum
{
  IR_CONTROL_TARGET_CHASSIS = 0,
  IR_CONTROL_TARGET_X,
  IR_CONTROL_TARGET_Z,
  IR_CONTROL_TARGET_SERVO_1,
  IR_CONTROL_TARGET_SERVO_2
} InfraredControlTarget;

/* 初始化 TIM4_CH4 红外接收；TIM4 计数频率必须为 1 MHz。 */
HAL_StatusTypeDef InfraredRemote_Init(TIM_HandleTypeDef *htim);

/* 在 HAL_TIM_IC_CaptureCallback 中转发 TIM4 的捕获事件。 */
void InfraredRemote_HandleCapture(TIM_HandleTypeDef *htim);

/* 在普通任务上下文中执行已解码的遥控命令。 */
void InfraredRemote_Process(void);
/* 页面变化后同步控制对象；总览/系统页会停止手动运动。 */
void InfraredRemote_SyncToCurrentPage(void);

uint8_t InfraredRemote_GetLastCommand(void);
InfraredControlAxis InfraredRemote_GetSelectedAxis(void);
InfraredMotionState InfraredRemote_GetSelectedMotionState(void);
uint8_t InfraredRemote_GetSelectedDirectionReverse(void);
uint8_t InfraredRemote_IsDirectionChangePending(void);
InfraredControlTarget InfraredRemote_GetSelectedTarget(void);
uint8_t InfraredRemote_GetSelectedServoIndex(void);
uint32_t InfraredRemote_GetPulseInput(void);

#endif
