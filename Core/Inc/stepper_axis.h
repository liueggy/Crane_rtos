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
/* 非阻塞输出指定脉冲数；reverse=0/1对应当前定义的正向/反向。 */
HAL_StatusTypeDef StepperAxis_MovePulses(StepperAxisId axis,
                                         uint32_t pulse_count,
                                         uint8_t reverse);
void StepperAxis_HandlePulseFinished(TIM_HandleTypeDef *timer);
uint8_t StepperAxis_IsPulseMoveActive(StepperAxisId axis);
uint32_t StepperAxis_GetCommandedPulses(StepperAxisId axis);
uint32_t StepperAxis_GetCompletedPulses(StepperAxisId axis);
uint32_t StepperAxis_GetRemainingPulses(StepperAxisId axis);
int32_t StepperAxis_GetPositionPulses(StepperAxisId axis);
uint8_t StepperAxis_ResetPositionPulses(StepperAxisId axis);
/* 停止状态下写入经过机械参考确认的绝对脉冲坐标。 */
uint8_t StepperAxis_SetPositionPulses(StepperAxisId axis, int32_t position);
uint8_t StepperAxis_IsEnabled(StepperAxisId axis);
uint8_t StepperAxis_IsHolding(StepperAxisId axis);
void StepperAxis_SetHoldWhenStopped(StepperAxisId axis, uint8_t enabled);
/* 已知上电姿态处于限位时，只授权指定轴沿触发方向的反方向脱离。 */
uint8_t StepperAxis_ArmPhotoLimitEscape(StepperAxisId axis,
                                        uint8_t blocked_direction_reverse);
uint8_t StepperAxis_IsPhotoLimitOwnedBy(StepperAxisId axis);
/* 在任务上下文处理PB11共用光电门的方向限位；X/Z运动强制互斥。 */
void StepperAxis_ProcessPhotoInterlock(void);
/* 安全停机：关闭脉冲并释放两路驱动器，不保留静态转矩。 */
void StepperAxis_StopAll(void);
/* 停止脉冲但维持Z轴静态保持；用于任务故障和软件急停。 */
void StepperAxis_StopMotionPreserveZ(void);
void StepperAxis_UpdateTelemetry(void);

#endif
