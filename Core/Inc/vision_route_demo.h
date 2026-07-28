#ifndef VISION_ROUTE_DEMO_H
#define VISION_ROUTE_DEMO_H

#include "k230_link.h"

#include <stdint.h>

typedef enum
{
  VISION_ROUTE_DEMO_IDLE = 0,
  VISION_ROUTE_DEMO_LIFT_Z,
  VISION_ROUTE_DEMO_HOME_X,
  VISION_ROUTE_DEMO_ALIGN_X,
  VISION_ROUTE_DEMO_MOVE_NUMBER,
  VISION_ROUTE_DEMO_SCAN_NUMBER_START,
  VISION_ROUTE_DEMO_SCAN_NUMBER_ROW,
  VISION_ROUTE_DEMO_SCAN_NUMBER_SIDE,
  VISION_ROUTE_DEMO_MOVE_BEAN,
  VISION_ROUTE_DEMO_SCAN_BEAN_A,
  VISION_ROUTE_DEMO_SCAN_BEAN_B,
  VISION_ROUTE_DEMO_COMPLETE,
  VISION_ROUTE_DEMO_FAULT
} VisionRouteDemoState;

void VisionRouteDemo_Init(void);
void VisionRouteDemo_Process(void);
void VisionRouteDemo_ToggleRunning(void);
void VisionRouteDemo_Abort(void);

VisionRouteDemoState VisionRouteDemo_GetState(void);
uint8_t VisionRouteDemo_IsRunning(void);
uint8_t VisionRouteDemo_GetDisplayResult(K230VisionResult *result);
uint8_t VisionRouteDemo_GetResultWarning(void);

#endif
