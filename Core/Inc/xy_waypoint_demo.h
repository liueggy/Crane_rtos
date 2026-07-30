#ifndef XY_WAYPOINT_DEMO_H
#define XY_WAYPOINT_DEMO_H

#include "world_map.h"

#include <stdint.h>

typedef enum
{
  XY_WAYPOINT_A = 0,
  XY_WAYPOINT_B,
  XY_WAYPOINT_C,
  XY_WAYPOINT_NUMBER_1,
  XY_WAYPOINT_NUMBER_2,
  XY_WAYPOINT_NUMBER_3,
  XY_WAYPOINT_NUMBER_4,
  XY_WAYPOINT_NUMBER_5,
  XY_WAYPOINT_COUNT
} XyWaypointId;

typedef enum
{
  XY_WAYPOINT_DEMO_IDLE = 0,
  XY_WAYPOINT_DEMO_REFERENCE,
  XY_WAYPOINT_DEMO_READY,
  XY_WAYPOINT_DEMO_MOVE_Y,
  XY_WAYPOINT_DEMO_CENTER_X_SETTLE,
  XY_WAYPOINT_DEMO_CENTER_X_MOVE,
  XY_WAYPOINT_DEMO_X_SETTLE,
  XY_WAYPOINT_DEMO_MOVE_X,
  XY_WAYPOINT_DEMO_COMPLETE,
  XY_WAYPOINT_DEMO_FAULT,
} XyWaypointDemoState;

void XyWaypointDemo_Init(void);
void XyWaypointDemo_Process(void);
void XyWaypointDemo_Select(XyWaypointId target);
void XyWaypointDemo_HandlePower(void);
void XyWaypointDemo_Abort(void);

XyWaypointDemoState XyWaypointDemo_GetState(void);
XyWaypointId XyWaypointDemo_GetSelected(void);
WorldStationId XyWaypointDemo_GetCurrentStation(void);
const char *XyWaypointDemo_GetCurrentName(void);
uint8_t XyWaypointDemo_HasReference(void);
uint8_t XyWaypointDemo_IsRunning(void);
const char *XyWaypointDemo_GetName(XyWaypointId target);

#endif
