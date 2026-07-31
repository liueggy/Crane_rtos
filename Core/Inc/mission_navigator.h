#ifndef MISSION_NAVIGATOR_H
#define MISSION_NAVIGATOR_H

#include "world_map.h"

#include <stdint.h>

typedef enum
{
  MISSION_NAV_IDLE = 0,
  MISSION_NAV_PREALIGN_X,
  MISSION_NAV_MOVE_Y,
  MISSION_NAV_MOVE_Y_CROSS_X,
  MISSION_NAV_CROSS_X,
  MISSION_NAV_DONE,
  MISSION_NAV_FAULT
} MissionNavigatorState;

typedef enum
{
  MISSION_NAV_RECOVERY_NONE = 0,
  MISSION_NAV_RECOVERY_RUNNING,
  MISSION_NAV_RECOVERY_FAILED
} MissionNavigatorRecoveryState;

typedef enum
{
  MISSION_NAV_CROSS_PHASE_NONE = 0,
  MISSION_NAV_CROSS_PHASE_WAIT_ENTRY,
  MISSION_NAV_CROSS_PHASE_WAIT_CLEAR,
  MISSION_NAV_CROSS_PHASE_MOVING_X,
  MISSION_NAV_CROSS_PHASE_WAIT_X_AT_START
} MissionNavigatorCrossPhase;

typedef enum
{
  MISSION_PAYLOAD_UNKNOWN = 0,
  MISSION_PAYLOAD_EMPTY,
  MISSION_PAYLOAD_LOADED
} MissionPayloadState;

typedef enum
{
  MISSION_LANE_NONE = 0,
  MISSION_LANE_LEFT,
  MISSION_LANE_RIGHT
} MissionRouteLane;

typedef enum
{
  MISSION_ROUTE_SAME_ZONE = 0,
  MISSION_ROUTE_LOADED_CENTER,
  MISSION_ROUTE_EMPTY_LEFT_DIRECT,
  MISSION_ROUTE_EMPTY_RIGHT_DIRECT,
  MISSION_ROUTE_EMPTY_CENTER_CROSS
} MissionRouteType;

typedef enum
{
  MISSION_ROUTE_PLAN_OK = 0,
  MISSION_ROUTE_PLAN_INVALID_POSE,
  MISSION_ROUTE_PLAN_UNKNOWN_PAYLOAD,
  MISSION_ROUTE_PLAN_NO_SAFE_PATH
} MissionRoutePlanFault;

typedef struct
{
  MissionPayloadState payload;
  MissionRouteLane entry_lane;
  MissionRouteLane exit_lane;
  MissionRouteType type;
  MissionRoutePlanFault fault;
  uint32_t x_cost_pulses;
  uint8_t requires_center_cross;
  uint8_t requires_prealign;
} MissionRoutePlan;

void MissionNavigator_Init(void);
uint8_t MissionNavigator_Start(WorldSlotId target_slot);
uint8_t MissionNavigator_StartWithPayload(WorldSlotId target_slot,
                                          MissionPayloadState payload);
uint8_t MissionNavigator_StartStation(WorldStationId target_station,
                                      int32_t target_x_pulses);
uint8_t MissionNavigator_StartStationWithPayload(
    WorldStationId target_station, int32_t target_x_pulses,
    MissionPayloadState payload);
void MissionNavigator_Process(void);
void MissionNavigator_Abort(void);
MissionNavigatorState MissionNavigator_GetState(void);
WorldStationId MissionNavigator_GetTargetStation(void);
MissionNavigatorRecoveryState MissionNavigator_GetRecoveryState(void);
MissionNavigatorCrossPhase MissionNavigator_GetCrossPhase(void);
const MissionRoutePlan *MissionNavigator_GetRoutePlan(void);

#endif
