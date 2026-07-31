#include "robot_controller.h"

#include "app_config.h"
#include "app_state.h"
#include "camera_tilt.h"
#include "chassis_motion.h"
#include "mission_action.h"
#include "mission_navigator.h"
#include "photo_sensor.h"
#include "robot_routes.h"
#include "route_executor.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "vision_route_demo.h"
#include "world_map.h"

#include <string.h>

static RobotState g_state;
static RobotFaultCode g_fault;
static uint8_t g_task_index;
static uint8_t g_start_requested;
static uint8_t g_action_started;
static uint8_t g_finish_stage;
static uint8_t g_runtime_rescan_used;
static MissionPayloadState g_payload;
static uint32_t g_missing_calibration;
static MissionActionFaultCode g_action_fault;
static RobotFaultRecord g_fault_history[ROBOT_FAULT_HISTORY_SIZE];
static uint8_t g_fault_history_head;
static uint8_t g_fault_history_count;
static uint8_t g_stage_retry_count;
static uint8_t g_recovery_pending;
static uint32_t g_recovery_deadline;

static void RecordFault(RobotFaultCode fault, uint8_t retry_count,
                        uint8_t recovered)
{
  RobotFaultRecord *record = &g_fault_history[g_fault_history_head];
  record->tick_ms = HAL_GetTick();
  record->state = g_state;
  record->fault = fault;
  record->action_fault = g_action_fault;
  record->retry_count = retry_count;
  record->recovered = recovered;
  g_fault_history_head = (uint8_t)((g_fault_history_head + 1U) %
                                   ROBOT_FAULT_HISTORY_SIZE);
  if (g_fault_history_count < ROBOT_FAULT_HISTORY_SIZE) ++g_fault_history_count;
}

static void StopMotion(void)
{
  RouteExecutor_Abort();
  MissionNavigator_Abort();
  MissionAction_Abort();
  VisionRouteDemo_Abort();
  ChassisMotion_Stop();
  StepperAxis_StopMotionPreserveZ();
  ServoControl_CancelSlew(0U);
  ServoControl_CancelSlew(1U);
}

static void EnterState(RobotState state)
{
  if ((state != g_state) &&
      !((g_state == ROBOT_STATE_TRANSPORT_VIA_CENTER) &&
        (state == ROBOT_STATE_MOVE_TO_DROP)))
  {
    g_stage_retry_count = 0U;
    g_recovery_pending = 0U;
  }
  g_state = state;
  g_action_started = 0U;
  if (state == ROBOT_STATE_FAULT)
  {
    StopMotion();
    AppState_SetMode(APP_MODE_FAULT);
  }
  else if ((state == ROBOT_STATE_IDLE) ||
           (state == ROBOT_STATE_CALIBRATION_REQUIRED) ||
           (state == ROBOT_STATE_FINISHED)) AppState_SetMode(APP_MODE_IDLE);
  else AppState_SetMode(APP_MODE_AUTO);
}

static void EnterFault(RobotFaultCode fault)
{
  g_action_fault = (fault == ROBOT_FAULT_ACTION) ?
                   MissionAction_GetFaultCode() : MISSION_ACTION_FAULT_NONE;
  if ((fault == ROBOT_FAULT_ACTION) &&
      (g_action_fault == MISSION_ACTION_FAULT_NONE) &&
      ((SafetyManager_GetFlags() & SAFETY_FAULT_LIMIT) != 0U))
    g_action_fault = MISSION_ACTION_FAULT_SAFETY;
  RecordFault(fault, 0U, 0U);
  g_fault = fault;
  g_payload = MISSION_PAYLOAD_UNKNOWN;
  WorldMap_InvalidateAll();
  EnterState(ROBOT_STATE_FAULT);
}

static void RetryCurrentStage(RobotFaultCode fault)
{
  /* 运动中故障会使坐标或实际载荷状态不再可信，只允许对尚未启动成功的
   * 瞬态忙/资源冲突做重试，禁止盲目重复整段导航或抓放动作。 */
  if (!WorldMap_IsPoseValid() ||
      ((fault == ROBOT_FAULT_NAVIGATION) &&
       (MissionNavigator_GetState() == MISSION_NAV_FAULT)) ||
      ((fault == ROBOT_FAULT_ACTION) &&
       (MissionAction_GetState() == MISSION_ACTION_FAULT)))
  {
    EnterFault(fault);
    return;
  }

  if (g_stage_retry_count >= 2U)
  {
    EnterFault(fault);
    return;
  }

  g_action_fault = (fault == ROBOT_FAULT_ACTION) ?
                   MissionAction_GetFaultCode() : MISSION_ACTION_FAULT_NONE;
  ++g_stage_retry_count;
  RecordFault(fault, g_stage_retry_count, 1U);
  MissionNavigator_Abort();
  MissionAction_Abort();
  ChassisMotion_Stop();
  StepperAxis_StopMotionPreserveZ();
  ServoControl_CancelSlew(0U);
  ServoControl_CancelSlew(1U);
  g_action_started = 0U;
  g_recovery_pending = 1U;
  g_recovery_deadline = HAL_GetTick() + 300U;
  if (g_state == ROBOT_STATE_MOVE_TO_DROP)
    g_state = ROBOT_STATE_TRANSPORT_VIA_CENTER;
}

static uint32_t MissingCalibration(void)
{
  uint32_t missing = 0U;
  if (!WorldMap_IsTopologyTrusted()) missing |= ROBOT_CAL_MISSING_WORLD;
  if (!WorldMap_IsStartupMotionCalibrated()) missing |= ROBOT_CAL_MISSING_SURVEY;
  if (!WorldMap_IsTaskMotionCalibrated()) missing |= ROBOT_CAL_MISSING_TASK;
  if (!RobotRoutes_IsCalibrated(ROBOT_ROUTE_SURVEY_START_TO_NUMBER) ||
      !RobotRoutes_IsCalibrated(ROBOT_ROUTE_SURVEY_NUMBER_TO_BEAN_DIRECT))
    missing |= ROBOT_CAL_MISSING_ROUTES;
  return missing;
}

static uint8_t StartPoseIsValid(void)
{
  return !ChassisMotion_IsRunning() &&
         !StepperAxis_IsEnabled(STEPPER_AXIS_X) &&
         !StepperAxis_IsEnabled(STEPPER_AXIS_Z) &&
         (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U) &&
         (PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_ORIGIN_SIDE) != 0U) &&
         (PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_FAR_SIDE) != 0U);
}

static uint8_t MoveAxisTo(StepperAxisId axis, int32_t target)
{
  int32_t current = StepperAxis_GetPositionPulses(axis);
  uint32_t pulses;
  if (StepperAxis_IsPulseMoveActive(axis)) return 1U;
  if (current == target) return 2U;
  pulses = (uint32_t)((current > target) ? current - target : target - current);
  return (StepperAxis_MovePulses(axis, pulses,
          (current > target) ? 1U : 0U) == HAL_OK) ? 1U : 0U;
}

static const MissionTransportTask *ActiveTask(void)
{
  return MissionPlanner_GetTask(g_task_index);
}

void RobotController_Init(void)
{
  WorldMap_Init();
  (void)StepperAxis_SetPositionPulses(
      STEPPER_AXIS_X, (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U));
  (void)StepperAxis_SetPositionPulses(
      STEPPER_AXIS_Z, (int32_t)STEPPER_Z_TRAVEL_PULSES);
  RobotRoutes_Init();
  RouteExecutor_Init();
  MissionNavigator_Init();
  MissionAction_Init();
  MissionPlanner_Init();
  g_task_index = 0U;
  g_start_requested = 0U;
  g_finish_stage = 0U;
  g_runtime_rescan_used = 0U;
  g_payload = MISSION_PAYLOAD_EMPTY;
  g_fault = ROBOT_FAULT_NONE;
  g_action_fault = MISSION_ACTION_FAULT_NONE;
  memset(g_fault_history, 0, sizeof(g_fault_history));
  g_fault_history_head = 0U;
  g_fault_history_count = 0U;
  g_stage_retry_count = 0U;
  g_recovery_pending = 0U;
  g_missing_calibration = MissingCalibration();
  EnterState(ROBOT_STATE_IDLE);
}

void RobotController_RequestStart(void)
{
  if ((g_state == ROBOT_STATE_IDLE) ||
      (g_state == ROBOT_STATE_FINISHED) ||
      (g_state == ROBOT_STATE_CALIBRATION_REQUIRED))
  {
    g_fault = ROBOT_FAULT_NONE;
    g_start_requested = 1U;
  }
}

void RobotController_RequestAbort(void)
{
  g_start_requested = 0U;
  EnterFault(ROBOT_FAULT_ACTION);
}

void RobotController_Update(void)
{
  const MissionTransportTask *task;
  uint8_t motion;

  StepperAxis_ProcessPhotoInterlock();
  RouteExecutor_Update();
  MissionNavigator_Process();
  MissionAction_Process();

  if ((SafetyManager_GetFlags() &
       (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) != 0U)
  {
    if (g_state != ROBOT_STATE_FAULT)
      EnterFault(SafetyManager_IsEstopActive() ?
                 ROBOT_FAULT_ESTOP : ROBOT_FAULT_ACTION);
    return;
  }

  if (g_recovery_pending)
  {
    if ((int32_t)(HAL_GetTick() - g_recovery_deadline) < 0) return;
    g_recovery_pending = 0U;
  }

  switch (g_state)
  {
    case ROBOT_STATE_IDLE:
    case ROBOT_STATE_FINISHED:
      if (g_start_requested) EnterState(ROBOT_STATE_SELF_CHECK);
      break;

    case ROBOT_STATE_SELF_CHECK:
      g_start_requested = 0U;
      g_missing_calibration = MissingCalibration();
      if (g_missing_calibration != 0U)
      {
        g_fault = ROBOT_FAULT_CALIBRATION;
        EnterState(ROBOT_STATE_CALIBRATION_REQUIRED);
      }
      else if (!StartPoseIsValid()) EnterFault(ROBOT_FAULT_START_POSE);
      else
      {
        WorldMap_SetPoseAtStart();
        g_task_index = 0U;
        g_runtime_rescan_used = 0U;
        g_payload = MISSION_PAYLOAD_EMPTY;
        if (!VisionRouteDemo_StartCompetition()) EnterFault(ROBOT_FAULT_VISION);
        else EnterState(ROBOT_STATE_MOVE_TO_NUMBER_SCAN);
      }
      break;

    case ROBOT_STATE_CALIBRATION_REQUIRED:
      if (g_start_requested) EnterState(ROBOT_STATE_SELF_CHECK);
      break;

    case ROBOT_STATE_MOVE_TO_NUMBER_SCAN:
      if (VisionRouteDemo_GetState() == VISION_ROUTE_DEMO_FAULT)
        EnterFault(ROBOT_FAULT_VISION);
      else if (VisionRouteDemo_IsComplete()) EnterState(ROBOT_STATE_TASK_BUILD);
      break;

    case ROBOT_STATE_TASK_BUILD:
      if (!MissionPlanner_Build(VisionRouteDemo_GetSurveyMap()))
        EnterFault(ROBOT_FAULT_TASK_MAP);
      else if (MissionPlanner_HasCompleteThreeTasks() &&
               (MissionPlanner_GetTaskCount() == MISSION_TRANSPORT_TASK_COUNT))
      {
        if (MissionPlanner_GetIssue() != MISSION_PLAN_OK)
          RecordFault(ROBOT_FAULT_TASK_MAP, 0U, 1U);
        EnterState(ROBOT_STATE_PREPARE_PICK);
      }
      else
        /* 仅保留结构损坏类兜底；普通缺识别已由规划器唯一补全。 */
        EnterFault(ROBOT_FAULT_TASK_MAP);
      break;

    case ROBOT_STATE_PREPARE_PICK:
      motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
      if (motion == 2U)
      {
        WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X),
                                 1U, 0, 1U);
        EnterState(ROBOT_STATE_MOVE_TO_PICK);
      }
      else if (motion == 0U) EnterFault(ROBOT_FAULT_Z_COORDINATE);
      break;

    case ROBOT_STATE_MOVE_TO_PICK:
      task = ActiveTask();
      if (task == 0) EnterFault(ROBOT_FAULT_TASK_MAP);
      else if (g_payload != MISSION_PAYLOAD_EMPTY)
        EnterFault(ROBOT_FAULT_ACTION);
      else if (!g_action_started)
      {
        g_action_started = MissionNavigator_StartWithPayload(
            task->pickup_slot, g_payload);
        if (!g_action_started) RetryCurrentStage(ROBOT_FAULT_NAVIGATION);
      }
      else if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        EnterState(ROBOT_STATE_PICK_ACTION);
      else if (MissionNavigator_GetState() == MISSION_NAV_FAULT)
        RetryCurrentStage(ROBOT_FAULT_NAVIGATION);
      break;

    case ROBOT_STATE_PICK_ACTION:
      task = ActiveTask();
      if (task == 0) EnterFault(ROBOT_FAULT_TASK_MAP);
      else if (!g_action_started)
      {
        g_action_started = MissionAction_StartPickup(task->pickup_slot);
        if (!g_action_started) RetryCurrentStage(ROBOT_FAULT_ACTION);
      }
      else if (MissionAction_GetState() == MISSION_ACTION_DONE)
      {
        AppConfig config;
        AppConfig_GetSnapshot(&config);
        if ((StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) != 0) ||
            (StepperAxis_GetRemainingPulses(STEPPER_AXIS_Z) != 0U) ||
            !WorldMap_GetPose()->z_valid ||
            (ServoControl_GetAngle(1U) != config.gripper_closed_degrees))
          EnterFault(ROBOT_FAULT_ACTION);
        else
        {
          g_payload = MISSION_PAYLOAD_LOADED;
          EnterState(ROBOT_STATE_LIFT_SAFE);
        }
      }
      else if (MissionAction_GetState() == MISSION_ACTION_FAULT)
        RetryCurrentStage(ROBOT_FAULT_ACTION);
      break;

    case ROBOT_STATE_LIFT_SAFE:
      if ((StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) != 0) ||
          (StepperAxis_GetRemainingPulses(STEPPER_AXIS_Z) != 0U) ||
          !WorldMap_GetPose()->z_valid) EnterFault(ROBOT_FAULT_Z_COORDINATE);
      else EnterState(ROBOT_STATE_TRANSPORT_VIA_CENTER);
      break;

    case ROBOT_STATE_TRANSPORT_VIA_CENTER:
      task = ActiveTask();
      if (task == 0) EnterFault(ROBOT_FAULT_TASK_MAP);
      else if (g_payload != MISSION_PAYLOAD_LOADED)
        EnterFault(ROBOT_FAULT_ACTION);
      else if (!MissionNavigator_StartWithPayload(
                   task->drop_slot, g_payload))
        RetryCurrentStage(ROBOT_FAULT_NAVIGATION);
      else EnterState(ROBOT_STATE_MOVE_TO_DROP);
      break;

    case ROBOT_STATE_MOVE_TO_DROP:
      if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        EnterState(ROBOT_STATE_DROP_ACTION);
      else if (MissionNavigator_GetState() == MISSION_NAV_FAULT)
        RetryCurrentStage(ROBOT_FAULT_NAVIGATION);
      break;

    case ROBOT_STATE_DROP_ACTION:
      task = ActiveTask();
      if (task == 0) EnterFault(ROBOT_FAULT_TASK_MAP);
      else if (!g_action_started)
      {
        g_action_started = MissionAction_StartDrop(task->drop_slot);
        if (!g_action_started) RetryCurrentStage(ROBOT_FAULT_ACTION);
      }
      else if (MissionAction_GetState() == MISSION_ACTION_DONE)
      {
        AppConfig config;
        AppConfig_GetSnapshot(&config);
        if ((StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) != 0) ||
            (StepperAxis_GetRemainingPulses(STEPPER_AXIS_Z) != 0U) ||
            !WorldMap_GetPose()->z_valid ||
            (ServoControl_GetAngle(1U) != config.gripper_closed_degrees) ||
            ServoControl_IsSlewActive(0U))
          EnterFault(ROBOT_FAULT_ACTION);
        else
        {
          g_payload = MISSION_PAYLOAD_EMPTY;
          MissionPlanner_MarkTaskCompleted(g_task_index);
          if (g_task_index + 1U < MissionPlanner_GetTaskCount())
          {
            ++g_task_index;
            EnterState(ROBOT_STATE_RETURN_TO_BEAN);
          }
          else if ((MissionPlanner_GetSkippedBeanMask() != 0U) &&
                   !g_runtime_rescan_used)
          {
            g_runtime_rescan_used = 1U;
            EnterState(ROBOT_STATE_RETURN_TO_BEAN_RESCAN);
          }
          else
          {
            g_finish_stage = 0U;
            EnterState(ROBOT_STATE_RETURN_FINISH);
          }
        }
      }
      else if (MissionAction_GetState() == MISSION_ACTION_FAULT)
        RetryCurrentStage(ROBOT_FAULT_ACTION);
      break;

    case ROBOT_STATE_RETURN_TO_BEAN_RESCAN:
      if (g_payload != MISSION_PAYLOAD_EMPTY)
        EnterFault(ROBOT_FAULT_ACTION);
      else if (!g_action_started)
      {
        g_action_started = MissionNavigator_StartStationWithPayload(
            WORLD_STATION_UPPER_OBSTACLE_BEAN_SCAN,
            StepperAxis_GetPositionPulses(STEPPER_AXIS_X),
            g_payload);
        if (!g_action_started) RetryCurrentStage(ROBOT_FAULT_NAVIGATION);
      }
      else if (MissionNavigator_GetState() == MISSION_NAV_DONE)
      {
        if (!VisionRouteDemo_StartBeanRescan()) EnterFault(ROBOT_FAULT_VISION);
        else EnterState(ROBOT_STATE_BEAN_RESCAN);
      }
      else if (MissionNavigator_GetState() == MISSION_NAV_FAULT)
        RetryCurrentStage(ROBOT_FAULT_NAVIGATION);
      break;

    case ROBOT_STATE_BEAN_RESCAN:
      if (VisionRouteDemo_GetState() == VISION_ROUTE_DEMO_FAULT)
        EnterFault(ROBOT_FAULT_VISION);
      else if (VisionRouteDemo_IsComplete())
      {
        uint8_t completed = MissionPlanner_GetCompletedBeanMask();
        if (!MissionPlanner_RebuildRemaining(VisionRouteDemo_GetSurveyMap(),
                                             completed))
          EnterFault(ROBOT_FAULT_TASK_MAP);
        else if (MissionPlanner_GetTaskCount() > 0U)
        {
          g_task_index = 0U;
          EnterState(ROBOT_STATE_PREPARE_PICK);
        }
        else
        {
          g_finish_stage = 0U;
          EnterState(ROBOT_STATE_RETURN_FINISH);
        }
      }
      break;

    case ROBOT_STATE_RETURN_TO_BEAN:
      task = ActiveTask();
      if (task == 0) EnterFault(ROBOT_FAULT_TASK_MAP);
      else if (g_payload != MISSION_PAYLOAD_EMPTY)
        EnterFault(ROBOT_FAULT_ACTION);
      else if (!g_action_started)
      {
        g_action_started = MissionNavigator_StartWithPayload(
            task->pickup_slot, g_payload);
        if (!g_action_started) RetryCurrentStage(ROBOT_FAULT_NAVIGATION);
      }
      else if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        EnterState(ROBOT_STATE_PICK_ACTION);
      else if (MissionNavigator_GetState() == MISSION_NAV_FAULT)
        RetryCurrentStage(ROBOT_FAULT_NAVIGATION);
      break;

    case ROBOT_STATE_RETURN_FINISH:
      if (g_payload != MISSION_PAYLOAD_EMPTY)
        EnterFault(ROBOT_FAULT_ACTION);
      else if (g_finish_stage == 0U)
      {
        if (!g_action_started)
        {
          g_action_started = MissionNavigator_StartStationWithPayload(
              WORLD_STATION_START, (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U),
              g_payload);
          if (!g_action_started) RetryCurrentStage(ROBOT_FAULT_NAVIGATION);
        }
        else if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        {
          g_finish_stage = 1U;
          g_action_started = 0U;
        }
        else if (MissionNavigator_GetState() == MISSION_NAV_FAULT)
          RetryCurrentStage(ROBOT_FAULT_NAVIGATION);
      }
      else if (g_finish_stage == 1U)
      {
        motion = MoveAxisTo(STEPPER_AXIS_X,
                            (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U));
        if (motion == 2U)
        {
          AppConfig config;
          AppConfig_GetSnapshot(&config);
          WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X),
                                   1U, 0, 1U);
          ServoControl_SetAngle(0U, config.servo_initial_degrees[0]);
          ServoControl_SetAngle(1U, config.gripper_closed_degrees);
          CameraTilt_SetLevel();
          g_finish_stage = 2U;
        }
        else if (motion == 0U) EnterFault(ROBOT_FAULT_X_COORDINATE);
      }
      else if (g_finish_stage == 2U)
      {
        if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
        {
          EnterFault(ROBOT_FAULT_FINAL_HOME);
          break;
        }
        if (StepperAxis_MovePulses(STEPPER_AXIS_Z,
            STEPPER_Z_TRAVEL_PULSES, 0U) != HAL_OK)
          EnterFault(ROBOT_FAULT_FINAL_HOME);
        else g_finish_stage = 3U;
      }
      else if (g_finish_stage == 3U &&
               !StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z))
      {
        if ((PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) == 0U) ||
            !StepperAxis_SetPositionPulses(STEPPER_AXIS_Z,
                                           (int32_t)STEPPER_Z_TRAVEL_PULSES))
          EnterFault(ROBOT_FAULT_FINAL_HOME);
        else
        {
          WorldMap_SetAxisPosition(
              (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U), 1U,
              (int32_t)STEPPER_Z_TRAVEL_PULSES, 1U);
          WorldMap_SetKnownStation(WORLD_STATION_START);
          EnterState(ROBOT_STATE_FINISHED);
        }
      }
      break;

    case ROBOT_STATE_FAULT:
    default:
      break;
  }
}

RobotState RobotController_GetState(void)
{
  return g_state;
}

uint32_t RobotController_GetMissingCalibrationMask(void)
{
  return g_missing_calibration;
}

const MissionTransportTask *RobotController_GetActiveTask(void)
{
  return ActiveTask();
}

uint8_t RobotController_GetTaskIndex(void)
{
  return g_task_index;
}

uint8_t RobotController_GetTaskCount(void)
{
  return MissionPlanner_GetTaskCount();
}

uint8_t RobotController_GetSkippedBeanMask(void)
{
  return MissionPlanner_GetSkippedBeanMask();
}

uint8_t RobotController_GetCompletedBeanMask(void)
{
  return MissionPlanner_GetCompletedBeanMask();
}

MissionPayloadState RobotController_GetPayloadState(void)
{
  return g_payload;
}

MissionRouteType RobotController_GetRouteType(void)
{
  return MissionNavigator_GetRoutePlan()->type;
}

MissionPlanIssue RobotController_GetMissionPlanIssue(void)
{
  return MissionPlanner_GetIssue();
}

uint8_t RobotController_GetMissionPlanIssueMask(void)
{
  return MissionPlanner_GetIssueMask();
}

WorldStationId RobotController_GetCurrentStation(void)
{
  return WorldMap_GetPose()->station;
}

WorldStationId RobotController_GetTargetStation(void)
{
  const MissionTransportTask *task = ActiveTask();
  if (g_state == ROBOT_STATE_MOVE_TO_NUMBER_SCAN)
  {
    VisionRouteDemoState vision_state = VisionRouteDemo_GetState();
    return (vision_state >= VISION_ROUTE_DEMO_MOVE_BEAN) ?
           WORLD_STATION_UPPER_OBSTACLE_BEAN_SCAN :
           WORLD_STATION_LOWER_OBSTACLE_NUMBER_SCAN;
  }
  if ((g_state == ROBOT_STATE_RETURN_TO_BEAN_RESCAN) ||
      (g_state == ROBOT_STATE_BEAN_RESCAN))
    return WORLD_STATION_UPPER_OBSTACLE_BEAN_SCAN;
  if ((task != 0) && ((g_state == ROBOT_STATE_PREPARE_PICK) ||
      (g_state == ROBOT_STATE_MOVE_TO_PICK) ||
      (g_state == ROBOT_STATE_PICK_ACTION) ||
      (g_state == ROBOT_STATE_RETURN_TO_BEAN)))
    return WorldMap_GetSlot(task->pickup_slot)->station;
  if ((task != 0) && ((g_state == ROBOT_STATE_LIFT_SAFE) ||
      (g_state == ROBOT_STATE_TRANSPORT_VIA_CENTER) ||
      (g_state == ROBOT_STATE_MOVE_TO_DROP) ||
      (g_state == ROBOT_STATE_DROP_ACTION)))
    return WorldMap_GetSlot(task->drop_slot)->station;
  if (g_state == ROBOT_STATE_RETURN_FINISH) return WORLD_STATION_START;
  return MissionNavigator_GetTargetStation();
}

RobotFaultCode RobotController_GetFaultCode(void)
{
  return g_fault;
}

MissionActionFaultCode RobotController_GetActionFaultCode(void)
{
  return (g_state == ROBOT_STATE_FAULT) ? g_action_fault :
         MissionAction_GetFaultCode();
}

uint8_t RobotController_GetFaultRecordCount(void)
{
  return g_fault_history_count;
}

uint8_t RobotController_GetFaultRecord(uint8_t newest_index,
                                       RobotFaultRecord *record)
{
  uint8_t slot;
  if ((record == 0) || (newest_index >= g_fault_history_count)) return 0U;
  slot = (uint8_t)((g_fault_history_head + ROBOT_FAULT_HISTORY_SIZE - 1U -
                    newest_index) % ROBOT_FAULT_HISTORY_SIZE);
  *record = g_fault_history[slot];
  return 1U;
}

uint8_t RobotController_IsRecovering(void) { return g_recovery_pending; }
uint8_t RobotController_GetRetryCount(void) { return g_stage_retry_count; }

const char *RobotController_GetPhaseText(void)
{
  ChassisAlignmentState alignment = ChassisMotion_GetAlignmentState();
  MissionNavigatorCrossPhase cross_phase = MissionNavigator_GetCrossPhase();
  if (g_recovery_pending) return "自动恢复";
  if ((g_state == ROBOT_STATE_DROP_ACTION) &&
      ServoControl_IsSlewActive(0U)) return "SERVO50";
  if (MissionNavigator_GetState() == MISSION_NAV_PREALIGN_X)
    return "中线预对齐";
  if (cross_phase == MISSION_NAV_CROSS_PHASE_WAIT_ENTRY) return "前往安全区";
  if (cross_phase == MISSION_NAV_CROSS_PHASE_WAIT_CLEAR) return "等光束恢复";
  if (cross_phase == MISSION_NAV_CROSS_PHASE_MOVING_X) return "并行换边";
  if (cross_phase == MISSION_NAV_CROSS_PHASE_WAIT_X_AT_START) return "起点等X";
  if (alignment == CHASSIS_ALIGNMENT_SETTLING) return "停稳检查";
  if (alignment == CHASSIS_ALIGNMENT_RETURNING) return "10速往返对齐";
  if ((alignment == CHASSIS_ALIGNMENT_RECOVERY_SETTLING) ||
      (alignment == CHASSIS_ALIGNMENT_RECOVERING)) return "15速恢复";
  switch (g_state)
  {
    case ROBOT_STATE_SELF_CHECK: return "自检";
    case ROBOT_STATE_MOVE_TO_NUMBER_SCAN:
    case ROBOT_STATE_TASK_BUILD: return VisionRouteDemo_IsRetrying() ? "补扫" : "识别";
    case ROBOT_STATE_PREPARE_PICK:
    case ROBOT_STATE_MOVE_TO_PICK: return "前往抓取";
    case ROBOT_STATE_PICK_ACTION: return "抓取";
    case ROBOT_STATE_LIFT_SAFE:
    case ROBOT_STATE_TRANSPORT_VIA_CENTER:
    case ROBOT_STATE_MOVE_TO_DROP: return "过起点";
    case ROBOT_STATE_DROP_ACTION: return "放置";
    case ROBOT_STATE_RETURN_TO_BEAN: return "返回";
    case ROBOT_STATE_RETURN_TO_BEAN_RESCAN:
    case ROBOT_STATE_BEAN_RESCAN: return "补识别";
    case ROBOT_STATE_RETURN_FINISH: return "收尾";
    case ROBOT_STATE_FINISHED: return "完成";
    case ROBOT_STATE_FAULT: return "故障";
    case ROBOT_STATE_CALIBRATION_REQUIRED: return "缺少标定";
    case ROBOT_STATE_IDLE:
    default: return "待机";
  }
}
