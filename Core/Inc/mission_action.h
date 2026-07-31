#ifndef MISSION_ACTION_H
#define MISSION_ACTION_H

#include "world_map.h"

#include <stdint.h>

typedef enum
{
  MISSION_ACTION_IDLE = 0,
  MISSION_ACTION_LIFT,
  MISSION_ACTION_PREPARE_TOOL,
  MISSION_ACTION_MOVE_X,
  MISSION_ACTION_DESCEND,
  MISSION_ACTION_GRIP,
  MISSION_ACTION_RAISE,
  MISSION_ACTION_RESET_TOOL,
  MISSION_ACTION_DONE,
  MISSION_ACTION_FAULT
} MissionActionState;

typedef enum
{
  MISSION_ACTION_FAULT_NONE = 0,
  MISSION_ACTION_FAULT_SAFETY,
  MISSION_ACTION_FAULT_Z_TOP,
  MISSION_ACTION_FAULT_X_MOVE,
  MISSION_ACTION_FAULT_Z_MOVE,
  MISSION_ACTION_FAULT_PB11_OWNER,
  MISSION_ACTION_FAULT_SERVO_TIMEOUT,
  MISSION_ACTION_FAULT_PULSE_INCOMPLETE,
  MISSION_ACTION_FAULT_SLOT
} MissionActionFaultCode;

void MissionAction_Init(void);
uint8_t MissionAction_StartPickup(WorldSlotId slot);
uint8_t MissionAction_StartDrop(WorldSlotId slot);
void MissionAction_Process(void);
void MissionAction_Abort(void);
MissionActionState MissionAction_GetState(void);
WorldSlotId MissionAction_GetSlot(void);
MissionActionFaultCode MissionAction_GetFaultCode(void);

#endif
