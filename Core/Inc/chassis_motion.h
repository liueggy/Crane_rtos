#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>

typedef enum
{
  CHASSIS_MOTION_IDLE = 0,
  CHASSIS_MOTION_MANUAL_TEST,
  CHASSIS_MOTION_ROUTE
} ChassisMotionMode;

void ChassisMotion_Init(void);
void ChassisMotion_TaskStep(void);
void ChassisMotion_SelectNextTestGear(void);
void ChassisMotion_ToggleTestDirection(void);
void ChassisMotion_AdjustPidTarget(int16_t delta_rpm);
void ChassisMotion_Stop(void);
uint8_t ChassisMotion_StartRouteSegment(int16_t distance_mm, int16_t turn_deg,
                                        uint16_t speed_rpm, uint16_t timeout_ms);
uint8_t ChassisMotion_IsRouteSegmentDone(void);

#endif
