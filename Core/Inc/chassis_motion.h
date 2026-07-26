#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>

void ChassisMotion_Init(void);
void ChassisMotion_TaskStep(void);

void ChassisMotion_ToggleRunning(void);
void ChassisMotion_SetRunning(uint8_t running);
void ChassisMotion_StopManual(void);
uint8_t ChassisMotion_IsRunning(void);

uint8_t ChassisMotion_SetDirection(uint8_t reverse);
uint8_t ChassisMotion_IsDirectionReverse(void);
uint8_t ChassisMotion_ToggleClosedLoop(void);
uint8_t ChassisMotion_SetClosedLoop(uint8_t enabled);
uint8_t ChassisMotion_IsClosedLoop(void);
void ChassisMotion_AdjustTargetRpm(int16_t delta_rpm);
void ChassisMotion_SetTargetRpm(int16_t target_rpm);
int16_t ChassisMotion_GetTargetRpm(void);

void ChassisMotion_Stop(void);
uint8_t ChassisMotion_StartRouteSegment(int16_t distance_mm, int16_t turn_deg,
                                        uint16_t speed_rpm, uint16_t timeout_ms);
uint8_t ChassisMotion_IsRouteSegmentDone(void);

#endif
