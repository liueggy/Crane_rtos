#ifndef MISSION_PLANNER_H
#define MISSION_PLANNER_H

#include "vision_survey.h"
#include "world_map.h"

#include <stdint.h>

#define MISSION_TRANSPORT_TASK_COUNT 3U

typedef enum
{
  MISSION_SKIP_NONE = 0,
  MISSION_SKIP_BEAN_UNKNOWN,
  MISSION_SKIP_BEAN_CONFLICT,
  MISSION_SKIP_NUMBER_UNKNOWN,
  MISSION_SKIP_NUMBER_CONFLICT
} MissionTaskSkipReason;

typedef enum
{
  MISSION_PLAN_OK = 0,
  MISSION_PLAN_BEAN_MISSING,
  MISSION_PLAN_BEAN_CONFLICT,
  MISSION_PLAN_NUMBER_MISSING,
  MISSION_PLAN_NUMBER_CONFLICT,
  MISSION_PLAN_TASK_COUNT_INVALID
} MissionPlanIssue;

typedef struct
{
  uint8_t bean_semantic;
  uint8_t physical_bean_slot;
  WorldSlotId pickup_slot;
  uint8_t target_number;
  WorldSlotId drop_slot;
  uint8_t bean_inferred;
  uint8_t drop_inferred;
} MissionTransportTask;

void MissionPlanner_Init(void);
uint8_t MissionPlanner_Build(const VisionSurveyMap *survey);
uint8_t MissionPlanner_RebuildRemaining(const VisionSurveyMap *survey,
                                        uint8_t completed_bean_mask);
uint8_t MissionPlanner_IsReady(void);
const MissionTransportTask *MissionPlanner_GetTask(uint8_t index);
uint8_t MissionPlanner_GetTaskCount(void);
uint8_t MissionPlanner_GetSkippedBeanMask(void);
uint8_t MissionPlanner_GetCompletedBeanMask(void);
uint8_t MissionPlanner_HasCompleteThreeTasks(void);
MissionPlanIssue MissionPlanner_GetIssue(void);
uint8_t MissionPlanner_GetIssueMask(void);
uint8_t MissionPlanner_GetInferredBeanMask(void);
uint8_t MissionPlanner_GetInferredNumberMask(void);
MissionTaskSkipReason MissionPlanner_GetSkipReason(uint8_t physical_bean_slot);
void MissionPlanner_MarkTaskCompleted(uint8_t index);

#endif
