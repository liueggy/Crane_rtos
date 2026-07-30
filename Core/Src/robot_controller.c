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

static RobotState g_state;
static RobotFaultCode g_fault;
static uint8_t g_task_index;
static uint8_t g_start_requested;
static uint8_t g_action_started;
static uint8_t g_finish_stage;
static uint32_t g_missing_calibration;

static void StopMotion(void)
{
  RouteExecutor_Abort();
  MissionNavigator_Abort();
  MissionAction_Abort();
  VisionRouteDemo_Abort();
  ChassisMotion_Stop();
  StepperAxis_StopAll();
}

static void EnterState(RobotState state)
{
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
  g_fault = fault;
  WorldMap_InvalidateAll();
  EnterState(ROBOT_STATE_FAULT);
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
  g_fault = ROBOT_FAULT_NONE;
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
      else EnterState(ROBOT_STATE_PREPARE_PICK);
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
      else if (!g_action_started)
      {
        g_action_started = MissionNavigator_Start(task->pickup_slot);
        if (!g_action_started) EnterFault(ROBOT_FAULT_NAVIGATION);
      }
      else if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        EnterState(ROBOT_STATE_PICK_ACTION);
      else if (MissionNavigator_GetState() == MISSION_NAV_FAULT)
        EnterFault(ROBOT_FAULT_NAVIGATION);
      break;

    case ROBOT_STATE_PICK_ACTION:
      task = ActiveTask();
      if (task == 0) EnterFault(ROBOT_FAULT_TASK_MAP);
      else if (!g_action_started)
      {
        g_action_started = MissionAction_StartPickup(task->pickup_slot);
        if (!g_action_started) EnterFault(ROBOT_FAULT_ACTION);
      }
      else if (MissionAction_GetState() == MISSION_ACTION_DONE)
        EnterState(ROBOT_STATE_LIFT_SAFE);
      else if (MissionAction_GetState() == MISSION_ACTION_FAULT)
        EnterFault(ROBOT_FAULT_ACTION);
      break;

    case ROBOT_STATE_LIFT_SAFE:
      if ((StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) != 0) ||
          !WorldMap_GetPose()->z_valid) EnterFault(ROBOT_FAULT_Z_COORDINATE);
      else EnterState(ROBOT_STATE_TRANSPORT_VIA_CENTER);
      break;

    case ROBOT_STATE_TRANSPORT_VIA_CENTER:
      task = ActiveTask();
      if (task == 0) EnterFault(ROBOT_FAULT_TASK_MAP);
      else if (!MissionNavigator_Start(task->drop_slot))
        EnterFault(ROBOT_FAULT_NAVIGATION);
      else EnterState(ROBOT_STATE_MOVE_TO_DROP);
      break;

    case ROBOT_STATE_MOVE_TO_DROP:
      if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        EnterState(ROBOT_STATE_DROP_ACTION);
      else if (MissionNavigator_GetState() == MISSION_NAV_FAULT)
        EnterFault(ROBOT_FAULT_NAVIGATION);
      break;

    case ROBOT_STATE_DROP_ACTION:
      task = ActiveTask();
      if (task == 0) EnterFault(ROBOT_FAULT_TASK_MAP);
      else if (!g_action_started)
      {
        g_action_started = MissionAction_StartDrop(task->drop_slot);
        if (!g_action_started) EnterFault(ROBOT_FAULT_ACTION);
      }
      else if (MissionAction_GetState() == MISSION_ACTION_DONE)
      {
        if (g_task_index + 1U < MISSION_TRANSPORT_TASK_COUNT)
        {
          ++g_task_index;
          EnterState(ROBOT_STATE_RETURN_TO_BEAN);
        }
        else
        {
          g_finish_stage = 0U;
          EnterState(ROBOT_STATE_RETURN_FINISH);
        }
      }
      else if (MissionAction_GetState() == MISSION_ACTION_FAULT)
        EnterFault(ROBOT_FAULT_ACTION);
      break;

    case ROBOT_STATE_RETURN_TO_BEAN:
      task = ActiveTask();
      if (task == 0) EnterFault(ROBOT_FAULT_TASK_MAP);
      else if (!g_action_started)
      {
        g_action_started = MissionNavigator_Start(task->pickup_slot);
        if (!g_action_started) EnterFault(ROBOT_FAULT_NAVIGATION);
      }
      else if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        EnterState(ROBOT_STATE_PICK_ACTION);
      else if (MissionNavigator_GetState() == MISSION_NAV_FAULT)
        EnterFault(ROBOT_FAULT_NAVIGATION);
      break;

    case ROBOT_STATE_RETURN_FINISH:
      if (g_finish_stage == 0U)
      {
        if (!g_action_started)
        {
          g_action_started = MissionNavigator_StartStation(
              WORLD_STATION_START, (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U));
          if (!g_action_started) EnterFault(ROBOT_FAULT_NAVIGATION);
        }
        else if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        {
          g_finish_stage = 1U;
          g_action_started = 0U;
        }
        else if (MissionNavigator_GetState() == MISSION_NAV_FAULT)
          EnterFault(ROBOT_FAULT_NAVIGATION);
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

const char *RobotController_GetPhaseText(void)
{
  ChassisAlignmentState alignment = ChassisMotion_GetAlignmentState();
  if (alignment == CHASSIS_ALIGNMENT_SETTLING) return "停稳检查";
  if (alignment == CHASSIS_ALIGNMENT_RETURNING) return "15速回退对齐";
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
    case ROBOT_STATE_RETURN_FINISH: return "收尾";
    case ROBOT_STATE_FINISHED: return "完成";
    case ROBOT_STATE_FAULT: return "故障";
    case ROBOT_STATE_CALIBRATION_REQUIRED: return "缺少标定";
    case ROBOT_STATE_IDLE:
    default: return "待机";
  }
}
