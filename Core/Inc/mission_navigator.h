#ifndef MISSION_NAVIGATOR_H
#define MISSION_NAVIGATOR_H

#include "world_map.h"

#include <stdint.h>

typedef enum
{
  MISSION_NAV_IDLE = 0,
  MISSION_NAV_MOVE_Y,
  MISSION_NAV_CROSS_X,
  MISSION_NAV_DONE,
  MISSION_NAV_FAULT
} MissionNavigatorState;

void MissionNavigator_Init(void);
uint8_t MissionNavigator_Start(WorldSlotId target_slot);
uint8_t MissionNavigator_StartStation(WorldStationId target_station,
                                      int32_t target_x_pulses);
void MissionNavigator_Process(void);
void MissionNavigator_Abort(void);
MissionNavigatorState MissionNavigator_GetState(void);
WorldStationId MissionNavigator_GetTargetStation(void);

#endif
