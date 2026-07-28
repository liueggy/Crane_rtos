#ifndef MISSION_PLANNER_H
#define MISSION_PLANNER_H

#include "vision_survey.h"
#include "world_map.h"

#include <stdint.h>

#define MISSION_TRANSPORT_TASK_COUNT 3U

typedef struct
{
  uint8_t bean_semantic;
  WorldSlotId pickup_slot;
  uint8_t target_number;
  WorldSlotId drop_slot;
} MissionTransportTask;

void MissionPlanner_Init(void);
uint8_t MissionPlanner_Build(const VisionSurveyMap *survey);
uint8_t MissionPlanner_IsReady(void);
const MissionTransportTask *MissionPlanner_GetTask(uint8_t index);

#endif
