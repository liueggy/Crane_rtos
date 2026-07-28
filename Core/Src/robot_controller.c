#include "robot_controller.h"

#include "app_config.h"
#include "app_state.h"
#include "chassis_motion.h"
#include "k230_link.h"
#include "photo_sensor.h"
#include "robot_routes.h"
#include "route_executor.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "vision_survey.h"
#include "world_map.h"

static RobotState g_state;
static uint8_t g_task_index;
static uint8_t g_start_requested;
static uint8_t g_action_started;
static uint32_t g_missing_calibration;

static void StopMotion(void)
{
  RouteExecutor_Abort();
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
           (state == ROBOT_STATE_FINISHED))
  {
    AppState_SetMode(APP_MODE_IDLE);
  }
  else AppState_SetMode(APP_MODE_AUTO);
}

static uint32_t MissingCalibration(void)
{
  uint32_t missing = 0U;
  if (WorldMap_GetMissingCalibrationMask() != 0U) missing |= ROBOT_CAL_MISSING_WORLD;
  if (!WorldMap_IsSurveyCalibrated()) missing |= ROBOT_CAL_MISSING_SURVEY;
  if (!WorldMap_IsTaskMotionCalibrated()) missing |= ROBOT_CAL_MISSING_TASK;
  for (uint8_t route = 0U; route < ROBOT_ROUTE_COUNT; ++route)
  {
    if (!RobotRoutes_IsCalibrated((RobotRouteId)route) ||
        !RobotRoutes_HasRequiredCenterPass((RobotRouteId)route))
    {
      missing |= ROBOT_CAL_MISSING_ROUTES;
      break;
    }
  }
  return missing;
}

static uint8_t StartPulseMove(StepperAxisId axis, uint32_t pulses, uint8_t reverse)
{
  return StepperAxis_MovePulses(axis, pulses, reverse) == HAL_OK;
}

void RobotController_Init(void)
{
  WorldMap_Init();
  /* 比赛人工摆放姿态已知：X在中点、Z触底；先同步到底层脉冲坐标。 */
  (void)StepperAxis_SetPositionPulses(STEPPER_AXIS_X,
                                     (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U));
  (void)StepperAxis_SetPositionPulses(STEPPER_AXIS_Z,
                                     (int32_t)STEPPER_Z_TRAVEL_PULSES);
  RobotRoutes_Init();
  RouteExecutor_Init();
  VisionSurvey_Init();
  MissionPlanner_Init();
  g_task_index = 0U;
  g_start_requested = 0U;
  g_missing_calibration = MissingCalibration();
  EnterState(ROBOT_STATE_IDLE);
}

void RobotController_RequestStart(void)
{
  if (!SafetyManager_IsEstopActive()) g_start_requested = 1U;
}

void RobotController_RequestAbort(void)
{
  g_start_requested = 0U;
  EnterState(ROBOT_STATE_FAULT);
}

void RobotController_Update(void)
{
  if (SafetyManager_IsEstopActive())
  {
    EnterState(ROBOT_STATE_FAULT);
    return;
  }
  RouteExecutor_Update();

  switch (g_state)
  {
    case ROBOT_STATE_IDLE:
      if (g_start_requested) EnterState(ROBOT_STATE_SELF_CHECK);
      break;

    case ROBOT_STATE_SELF_CHECK:
      g_missing_calibration = MissingCalibration();
      if (g_missing_calibration != 0U)
      {
        g_start_requested = 0U;
        EnterState(ROBOT_STATE_CALIBRATION_REQUIRED);
      }
      else EnterState(ROBOT_STATE_STARTUP_CLEAR_Z);
      break;

    case ROBOT_STATE_CALIBRATION_REQUIRED:
      /* 调试阶段写入全部标定值后，可再次请求启动并重新自检。 */
      if (g_start_requested) EnterState(ROBOT_STATE_SELF_CHECK);
      break;

    case ROBOT_STATE_STARTUP_CLEAR_Z:
      if (!g_action_started)
      {
        /* 规定起点为Z轴触底；若PB11已遮挡，只授权Z轴向上脱离。 */
        (void)StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_Z, 0U);
        g_action_started = StartPulseMove(STEPPER_AXIS_Z,
                                          WorldMap_GetStartupLiftPulses(), 1U);
        if (!g_action_started) EnterState(ROBOT_STATE_FAULT);
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z))
      {
        const WorldPose *pose = WorldMap_GetPose();
        WorldMap_SetAxisPosition(pose->x_pulses, pose->x_valid,
            StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
        EnterState(ROBOT_STATE_STARTUP_HOME_X);
      }
      break;

    case ROBOT_STATE_STARTUP_HOME_X:
      if (!g_action_started)
      {
        /* Z挡片已离开PB11后，X向右端连续寻找光电零点。 */
        g_action_started = StartPulseMove(STEPPER_AXIS_X,
                                          STEPPER_X_SAFE_TRAVEL_PULSES, 1U);
        if (!g_action_started) EnterState(ROBOT_STATE_FAULT);
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X))
      {
        if ((PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) == 0U) ||
            !StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_X))
        {
          EnterState(ROBOT_STATE_FAULT);
          break;
        }
        (void)StepperAxis_SetPositionPulses(STEPPER_AXIS_X, 0);
        WorldMap_SetAxisPosition(0, 1U,
            StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
        EnterState(ROBOT_STATE_STARTUP_ALIGN_NUMBER_2);
      }
      break;

    case ROBOT_STATE_STARTUP_ALIGN_NUMBER_2:
      if (!g_action_started)
      {
        /* 从右端零点沿+X离开限位，并快速对准物理2号箱。 */
        (void)StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_X, 1U);
        g_action_started = StartPulseMove(STEPPER_AXIS_X,
                                          WorldMap_GetNumber2AlignmentPulses(), 0U);
        if (!g_action_started) EnterState(ROBOT_STATE_FAULT);
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X))
      {
        WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X), 1U,
            StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
        EnterState(ROBOT_STATE_MOVE_TO_NUMBER_SCAN);
      }
      break;

    case ROBOT_STATE_MOVE_TO_NUMBER_SCAN:
      if (!g_action_started)
      {
        g_action_started = RouteExecutor_Start(ROBOT_ROUTE_SURVEY_START_TO_NUMBER);
        if (!g_action_started) EnterState(ROBOT_STATE_FAULT);
      }
      else if (RouteExecutor_GetState() == ROUTE_EXECUTOR_DONE)
      {
        if (!VisionSurvey_Begin()) EnterState(ROBOT_STATE_CALIBRATION_REQUIRED);
        else
        {
          (void)K230Link_SelectTask(K230_TASK_NUMBER);
          EnterState(ROBOT_STATE_NUMBER_SCAN_A);
        }
      }
      break;

    case ROBOT_STATE_NUMBER_SCAN_A:
      /* 未来由扫描执行器到达A姿态，并向VisionSurvey提交稳定多帧结果。 */
      if (VisionSurvey_GetState() == VISION_SURVEY_NUMBER_B)
        EnterState(ROBOT_STATE_NUMBER_SCAN_B);
      break;

    case ROBOT_STATE_NUMBER_SCAN_B:
      if (VisionSurvey_GetState() == VISION_SURVEY_BEAN_C)
        EnterState(ROBOT_STATE_MOVE_TO_BEAN_SCAN);
      break;

    case ROBOT_STATE_MOVE_TO_BEAN_SCAN:
      if (!g_action_started)
      {
        g_action_started = RouteExecutor_Start(ROBOT_ROUTE_SURVEY_NUMBER_TO_BEAN_DIRECT);
        if (!g_action_started) EnterState(ROBOT_STATE_FAULT);
      }
      else if (RouteExecutor_GetState() == ROUTE_EXECUTOR_DONE)
      {
        (void)K230Link_SelectTask(K230_TASK_BEAN);
        EnterState(ROBOT_STATE_BEAN_SCAN_C);
      }
      break;

    case ROBOT_STATE_BEAN_SCAN_C:
      if (VisionSurvey_IsComplete()) EnterState(ROBOT_STATE_TASK_BUILD);
      break;

    case ROBOT_STATE_TASK_BUILD:
      if (!MissionPlanner_Build(VisionSurvey_GetMap())) EnterState(ROBOT_STATE_FAULT);
      else
      {
        g_task_index = 0U;
        EnterState(ROBOT_STATE_PREPARE_PICK);
      }
      break;

    case ROBOT_STATE_PREPARE_PICK:
      if (!g_action_started)
      {
        AppConfig config;
        int32_t z = StepperAxis_GetPositionPulses(STEPPER_AXIS_Z);
        int32_t prep_z = WorldMap_GetPickupPrepZPulses();
        /* 识别结束即张开夹爪；Z轴同时抬到最高抓取层上方预备点。 */
        AppConfig_GetSnapshot(&config);
        ServoControl_SetAngle(1U, config.gripper_open_degrees);
        if (z <= prep_z)
        {
          EnterState(ROBOT_STATE_MOVE_TO_PICK);
          break;
        }
        g_action_started = StartPulseMove(STEPPER_AXIS_Z,
                                          (uint32_t)(z - prep_z), 1U);
        if (!g_action_started) EnterState(ROBOT_STATE_FAULT);
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z))
      {
        WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X), 1U,
            StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
        EnterState(ROBOT_STATE_MOVE_TO_PICK);
      }
      break;

    case ROBOT_STATE_MOVE_TO_PICK:
      /* 根据任务pickup_slot选择顶端角点或S1，并移动到标定X位置。 */
      break;
    case ROBOT_STATE_PICK_ACTION:
      /* 先应用槽位tool_yaw（豆子箱均为初始0度），再下降到抓取Z高度。 */
      break;
    case ROBOT_STATE_LIFT_SAFE:
      /* 抬升到运输高度后才允许底盘运动。 */
      break;
    case ROBOT_STATE_TRANSPORT_VIA_CENTER:
      /* 正式搬运必须调用TRANSPORT_*_CENTER路线。 */
      break;
    case ROBOT_STATE_MOVE_TO_DROP:
      /* 根据任务drop_slot选择底端角点或S6，并移动到标定X位置。 */
      break;
    case ROBOT_STATE_DROP_ACTION:
      /* 先应用槽位tool_yaw（横排90度、两侧0度），再下降到Z=3300释放。 */
      break;
    case ROBOT_STATE_RETURN_TO_BEAN:
      /* 非最后一项使用NUMBER_TO_BEAN_CENTER路线返回豆子区。 */
      break;
    case ROBOT_STATE_RETURN_FINISH:
      /* 最后一项停留在数字区并收纳全部可移动机构。 */
      break;
    case ROBOT_STATE_FINISHED:
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
  return MissionPlanner_GetTask(g_task_index);
}
