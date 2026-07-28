#ifndef ROUTE_EXECUTOR_H
#define ROUTE_EXECUTOR_H

#include "robot_routes.h"

#include <stdint.h>

typedef enum
{
  ROUTE_EXECUTOR_IDLE = 0,
  ROUTE_EXECUTOR_RUNNING,
  ROUTE_EXECUTOR_DONE,
  ROUTE_EXECUTOR_NEEDS_CALIBRATION,
  ROUTE_EXECUTOR_FAULT
} RouteExecutorState;

void RouteExecutor_Init(void);
uint8_t RouteExecutor_Start(RobotRouteId route_id);
void RouteExecutor_Update(void);
void RouteExecutor_Abort(void);
RouteExecutorState RouteExecutor_GetState(void);
RobotRouteId RouteExecutor_GetRoute(void);
uint8_t RouteExecutor_GetSegmentIndex(void);

#endif
