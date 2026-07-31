#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "mission_planner.h"
#include "mission_navigator.h"
#include "mission_action.h"

#include <stdint.h>

typedef enum
{
  ROBOT_STATE_IDLE = 0,
  ROBOT_STATE_CALIBRATION_REQUIRED,
  ROBOT_STATE_SELF_CHECK,
  ROBOT_STATE_STARTUP_CLEAR_Z,
  ROBOT_STATE_STARTUP_HOME_X,
  ROBOT_STATE_STARTUP_ALIGN_NUMBER_2,
  ROBOT_STATE_MOVE_TO_NUMBER_SCAN,
  ROBOT_STATE_NUMBER_SCAN_A,
  ROBOT_STATE_NUMBER_SCAN_B,
  ROBOT_STATE_MOVE_TO_BEAN_SCAN,
  ROBOT_STATE_BEAN_SCAN_C,
  ROBOT_STATE_TASK_BUILD,
  ROBOT_STATE_PREPARE_PICK,
  ROBOT_STATE_MOVE_TO_PICK,
  ROBOT_STATE_PICK_ACTION,
  ROBOT_STATE_LIFT_SAFE,
  ROBOT_STATE_TRANSPORT_VIA_CENTER,
  ROBOT_STATE_MOVE_TO_DROP,
  ROBOT_STATE_DROP_ACTION,
  ROBOT_STATE_RETURN_TO_BEAN,
  ROBOT_STATE_RETURN_TO_BEAN_RESCAN,
  ROBOT_STATE_BEAN_RESCAN,
  ROBOT_STATE_RETURN_FINISH,
  ROBOT_STATE_FINISHED,
  ROBOT_STATE_FAULT
} RobotState;

typedef enum
{
  ROBOT_FAULT_NONE = 0,
  ROBOT_FAULT_ESTOP,
  ROBOT_FAULT_START_POSE,
  ROBOT_FAULT_CALIBRATION,
  ROBOT_FAULT_VISION,
  ROBOT_FAULT_TASK_MAP,
  ROBOT_FAULT_NAVIGATION,
  ROBOT_FAULT_ACTION,
  ROBOT_FAULT_X_COORDINATE,
  ROBOT_FAULT_Z_COORDINATE,
  ROBOT_FAULT_FINAL_HOME
} RobotFaultCode;

typedef enum
{
  ROBOT_CAL_MISSING_WORLD = (1UL << 0),
  ROBOT_CAL_MISSING_SURVEY = (1UL << 1),
  ROBOT_CAL_MISSING_TASK = (1UL << 2),
  ROBOT_CAL_MISSING_ROUTES = (1UL << 3)
} RobotCalibrationFlag;

void RobotController_Init(void);
void RobotController_RequestStart(void);
void RobotController_RequestAbort(void);
void RobotController_Update(void);
RobotState RobotController_GetState(void);
uint32_t RobotController_GetMissingCalibrationMask(void);
const MissionTransportTask *RobotController_GetActiveTask(void);
uint8_t RobotController_GetTaskIndex(void);
uint8_t RobotController_GetTaskCount(void);
uint8_t RobotController_GetSkippedBeanMask(void);
uint8_t RobotController_GetCompletedBeanMask(void);
MissionPayloadState RobotController_GetPayloadState(void);
MissionRouteType RobotController_GetRouteType(void);
MissionPlanIssue RobotController_GetMissionPlanIssue(void);
uint8_t RobotController_GetMissionPlanIssueMask(void);
WorldStationId RobotController_GetCurrentStation(void);
WorldStationId RobotController_GetTargetStation(void);
RobotFaultCode RobotController_GetFaultCode(void);
MissionActionFaultCode RobotController_GetActionFaultCode(void);
const char *RobotController_GetPhaseText(void);

#endif
