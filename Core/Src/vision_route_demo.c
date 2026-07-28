#include "vision_route_demo.h"

#include "app_config.h"
#include "camera_tilt.h"
#include "chassis_motion.h"
#include "initialization_debug.h"
#include "odometry_calibration.h"
#include "photo_sensor.h"
#include "robot_controller.h"
#include "route_executor.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "world_map.h"

#include <string.h>

#define VISION_DEMO_Z_TIMEOUT_MS          12000U
#define VISION_DEMO_X_HOME_TIMEOUT_MS     22000U
#define VISION_DEMO_X_MOVE_TIMEOUT_MS     16000U
#define VISION_DEMO_POINT_DWELL_MS          1000U
#define VISION_DEMO_DIRECTION_SETTLE_MS     300U
#define VISION_DEMO_NUMBER_YAW_DEGREES         0U
#define VISION_DEMO_NUMBER_SIDE_YAW_DEGREES   54U
#define VISION_DEMO_BEAN_A_YAW_DEGREES       180U
#define VISION_DEMO_BEAN_B_YAW_DEGREES       194U
#define VISION_DEMO_DISPLAY_WIDTH             640U
#define VISION_DEMO_STABLE_HITS                  2U

typedef struct
{
  uint8_t candidate;
  uint8_t hits;
  uint8_t stable;
  uint8_t confidence;
} VisionSlotVote;

static VisionRouteDemoState g_state;
static uint8_t g_stage_started;
static uint8_t g_result_warning;
static uint32_t g_deadline;
static uint32_t g_not_before_tick;
static uint16_t g_last_sequence;
static K230VisionResult g_display_result;
static VisionSlotVote g_number_votes[5];
static VisionSlotVote g_bean_votes[3];

static uint8_t DeadlineExpired(void)
{
  return ((int32_t)(HAL_GetTick() - g_deadline) >= 0) ? 1U : 0U;
}

static uint8_t TimeReached(uint32_t tick)
{
  return ((int32_t)(HAL_GetTick() - tick) >= 0) ? 1U : 0U;
}

static uint8_t HasSafetyFault(void)
{
  return (SafetyManager_GetFlags() &
          (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) ? 1U : 0U;
}

static void StopMotion(void)
{
  RouteExecutor_Abort();
  ChassisMotion_Stop();
  StepperAxis_SetEnabled(STEPPER_AXIS_X, 0U);
  StepperAxis_SetEnabled(STEPPER_AXIS_Z, 0U);
}

static void EnterState(VisionRouteDemoState state)
{
  g_state = state;
  g_stage_started = 0U;
}

static void EnterFault(void)
{
  StopMotion();
  EnterState(VISION_ROUTE_DEMO_FAULT);
}

static uint8_t SlotFromX(uint16_t center_x, uint8_t count)
{
  uint32_t slot = ((uint32_t)center_x * count) / VISION_DEMO_DISPLAY_WIDTH;
  if (slot >= count) slot = count - 1U;
  return (uint8_t)slot;
}

static uint8_t IsExpectedSemantic(K230VisionTask task, uint8_t semantic)
{
  if (task == K230_TASK_NUMBER)
    return (semantic >= K230_SEMANTIC_NUMBER_1) &&
           (semantic <= K230_SEMANTIC_NUMBER_5);
  return (semantic == K230_SEMANTIC_BEAN_L) ||
         (semantic == K230_SEMANTIC_BEAN_H) ||
         (semantic == K230_SEMANTIC_BEAN_B);
}

static void RebuildDisplayResult(K230VisionTask task, VisionSlotVote *votes,
                                 uint8_t slot_count)
{
  memset(&g_display_result, 0, sizeof(g_display_result));
  g_display_result.valid = 1U;
  g_display_result.task = (uint8_t)task;
  g_display_result.sequence = g_last_sequence;
  g_display_result.received_tick_ms = HAL_GetTick();
  for (uint8_t slot = 0U; slot < slot_count; ++slot)
  {
    if (!votes[slot].stable) continue;
    K230VisionTarget *target = &g_display_result.targets[g_display_result.count++];
    target->semantic = votes[slot].candidate;
    target->confidence_percent = votes[slot].confidence;
    target->center_x = (uint16_t)(((uint32_t)(slot * 2U + 1U) *
                                  VISION_DEMO_DISPLAY_WIDTH) /
                                 (slot_count * 2U));
  }
}

static void AcceptLatestFrame(K230VisionTask task)
{
  K230VisionResult result;
  VisionSlotVote *votes = (task == K230_TASK_NUMBER) ?
                          g_number_votes : g_bean_votes;
  uint8_t slot_count = (task == K230_TASK_NUMBER) ? 5U : 3U;

  if (!K230Link_GetLatestResult(&result) || !result.valid ||
      (result.task != (uint8_t)task) ||
      (result.sequence == g_last_sequence) ||
      !K230Link_IsResultFresh(1000U)) return;

  g_last_sequence = result.sequence;
  for (uint8_t i = 0U; i < result.count; ++i)
  {
    const K230VisionTarget *target = &result.targets[i];
    if (!IsExpectedSemantic(task, target->semantic)) continue;
    uint8_t slot = SlotFromX(target->center_x, slot_count);
    VisionSlotVote *vote = &votes[slot];

    if (vote->candidate == target->semantic)
    {
      if (vote->hits < 255U) ++vote->hits;
      if (target->confidence_percent > vote->confidence)
        vote->confidence = target->confidence_percent;
    }
    else if (!vote->stable &&
             ((vote->hits < VISION_DEMO_STABLE_HITS) ||
              (target->confidence_percent > vote->confidence)))
    {
      vote->candidate = target->semantic;
      vote->hits = 1U;
      vote->confidence = target->confidence_percent;
    }
    if (vote->hits >= VISION_DEMO_STABLE_HITS) vote->stable = 1U;
  }
  RebuildDisplayResult(task, votes, slot_count);
}

static uint8_t StartLiftFromKnownBottom(void)
{
  if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
  {
    if (!StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_Z) &&
        !StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_Z, 0U)) return 0U;
    if (!StepperAxis_SetPositionPulses(STEPPER_AXIS_Z,
                                      (int32_t)STEPPER_Z_TRAVEL_PULSES)) return 0U;
  }
  return StepperAxis_MovePulses(STEPPER_AXIS_Z,
                               WorldMap_GetStartupLiftPulses(), 1U) == HAL_OK;
}

void VisionRouteDemo_Init(void)
{
  memset(&g_display_result, 0, sizeof(g_display_result));
  memset(g_number_votes, 0, sizeof(g_number_votes));
  memset(g_bean_votes, 0, sizeof(g_bean_votes));
  g_result_warning = 0U;
  g_last_sequence = 0U;
  EnterState(VISION_ROUTE_DEMO_IDLE);
}

void VisionRouteDemo_Abort(void)
{
  StopMotion();
  EnterState(VISION_ROUTE_DEMO_IDLE);
}

void VisionRouteDemo_ToggleRunning(void)
{
  RobotState robot_state = RobotController_GetState();
  if (VisionRouteDemo_IsRunning())
  {
    VisionRouteDemo_Abort();
    return;
  }
  if (HasSafetyFault() || InitializationDebug_IsRunning() ||
      OdometryCalibration_IsRunning() ||
      ((robot_state != ROBOT_STATE_IDLE) &&
       (robot_state != ROBOT_STATE_CALIBRATION_REQUIRED) &&
       (robot_state != ROBOT_STATE_FINISHED)))
  {
    EnterFault();
    return;
  }

  StopMotion();
  memset(&g_display_result, 0, sizeof(g_display_result));
  memset(g_number_votes, 0, sizeof(g_number_votes));
  memset(g_bean_votes, 0, sizeof(g_bean_votes));
  g_result_warning = 0U;
  g_last_sequence = 0U;
  ServoControl_SetAngle(0U, VISION_DEMO_NUMBER_YAW_DEGREES);
  CameraTilt_SetLevel();
  EnterState(VISION_ROUTE_DEMO_LIFT_Z);
}

void VisionRouteDemo_Process(void)
{
  if (!VisionRouteDemo_IsRunning()) return;
  StepperAxis_ProcessPhotoInterlock();
  if (HasSafetyFault())
  {
    EnterFault();
    return;
  }

  switch (g_state)
  {
    case VISION_ROUTE_DEMO_LIFT_Z:
      if (!g_stage_started)
      {
        if (!StartLiftFromKnownBottom()) EnterFault();
        else
        {
          g_stage_started = 1U;
          g_deadline = HAL_GetTick() + VISION_DEMO_Z_TIMEOUT_MS;
        }
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z))
      {
        if (StepperAxis_GetRemainingPulses(STEPPER_AXIS_Z) != 0U) EnterFault();
        else
        {
          StepperAxis_SetDirectionReverse(STEPPER_AXIS_X, 1U);
          g_not_before_tick = HAL_GetTick() + VISION_DEMO_DIRECTION_SETTLE_MS;
          EnterState(VISION_ROUTE_DEMO_HOME_X);
        }
      }
      else if (DeadlineExpired()) EnterFault();
      break;

    case VISION_ROUTE_DEMO_HOME_X:
      if (!g_stage_started && TimeReached(g_not_before_tick))
      {
        if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
        {
          EnterFault();
          break;
        }
        StepperAxis_SetEnabled(STEPPER_AXIS_X, 1U);
        if (!StepperAxis_IsEnabled(STEPPER_AXIS_X)) EnterFault();
        else
        {
          g_stage_started = 1U;
          g_deadline = HAL_GetTick() + VISION_DEMO_X_HOME_TIMEOUT_MS;
        }
      }
      else if (g_stage_started &&
               (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U))
      {
        StepperAxis_ProcessPhotoInterlock();
        if (!StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_X) ||
            !StepperAxis_SetPositionPulses(STEPPER_AXIS_X, 0)) EnterFault();
        else
        {
          StepperAxis_SetDirectionReverse(STEPPER_AXIS_X, 0U);
          g_not_before_tick = HAL_GetTick() + VISION_DEMO_DIRECTION_SETTLE_MS;
          EnterState(VISION_ROUTE_DEMO_ALIGN_X);
        }
      }
      else if (g_stage_started && DeadlineExpired()) EnterFault();
      break;

    case VISION_ROUTE_DEMO_ALIGN_X:
      if (!g_stage_started && TimeReached(g_not_before_tick))
      {
        if (!StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_X, 1U) ||
            (StepperAxis_MovePulses(STEPPER_AXIS_X,
             WorldMap_GetNumber2AlignmentPulses(), 0U) != HAL_OK)) EnterFault();
        else
        {
          g_stage_started = 1U;
          g_deadline = HAL_GetTick() + VISION_DEMO_X_MOVE_TIMEOUT_MS;
        }
      }
      else if (g_stage_started &&
               !StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X))
      {
        if (StepperAxis_GetRemainingPulses(STEPPER_AXIS_X) != 0U) EnterFault();
        else EnterState(VISION_ROUTE_DEMO_MOVE_NUMBER);
      }
      else if (g_stage_started && DeadlineExpired()) EnterFault();
      break;

    case VISION_ROUTE_DEMO_MOVE_NUMBER:
      if (!g_stage_started)
      {
        g_stage_started = RouteExecutor_Start(ROBOT_ROUTE_SURVEY_START_TO_NUMBER);
        if (!g_stage_started) EnterFault();
      }
      else if (RouteExecutor_GetState() == ROUTE_EXECUTOR_DONE)
      {
        ServoControl_SetAngle(0U, VISION_DEMO_NUMBER_YAW_DEGREES);
        CameraTilt_SetLevel();
        K230Link_InvalidateResult();
        (void)K230Link_SelectTask(K230_TASK_NUMBER);
        g_deadline = HAL_GetTick() + VISION_DEMO_POINT_DWELL_MS;
        EnterState(VISION_ROUTE_DEMO_SCAN_NUMBER_START);
      }
      else if (RouteExecutor_GetState() == ROUTE_EXECUTOR_FAULT) EnterFault();
      break;

    case VISION_ROUTE_DEMO_SCAN_NUMBER_START:
      AcceptLatestFrame(K230_TASK_NUMBER);
      if (DeadlineExpired())
      {
        EnterState(VISION_ROUTE_DEMO_SCAN_NUMBER_ROW);
      }
      break;

    case VISION_ROUTE_DEMO_SCAN_NUMBER_ROW:
      AcceptLatestFrame(K230_TASK_NUMBER);
      if (!g_stage_started)
      {
        int32_t current_x = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
        int32_t target_x = NUMBER_SLOT_BOTTOM_LEFT_X_PULSES;
        if ((current_x >= target_x) ||
            (StepperAxis_MovePulses(STEPPER_AXIS_X,
             (uint32_t)(target_x - current_x), 0U) != HAL_OK)) EnterFault();
        else
        {
          g_stage_started = 1U;
          g_deadline = HAL_GetTick() + VISION_DEMO_X_MOVE_TIMEOUT_MS;
        }
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X))
      {
        if (StepperAxis_GetRemainingPulses(STEPPER_AXIS_X) != 0U) EnterFault();
        else
        {
          ServoControl_SetAngle(0U, VISION_DEMO_NUMBER_SIDE_YAW_DEGREES);
          g_deadline = HAL_GetTick() + VISION_DEMO_POINT_DWELL_MS;
          EnterState(VISION_ROUTE_DEMO_SCAN_NUMBER_SIDE);
        }
      }
      else if (DeadlineExpired()) EnterFault();
      break;

    case VISION_ROUTE_DEMO_SCAN_NUMBER_SIDE:
      AcceptLatestFrame(K230_TASK_NUMBER);
      if (DeadlineExpired())
      {
        ServoControl_SetAngle(0U, VISION_DEMO_BEAN_A_YAW_DEGREES);
        if (!RouteExecutor_Start(ROBOT_ROUTE_SURVEY_NUMBER_TO_BEAN_DIRECT))
          EnterFault();
        else EnterState(VISION_ROUTE_DEMO_MOVE_BEAN);
      }
      break;

    case VISION_ROUTE_DEMO_MOVE_BEAN:
      if (RouteExecutor_GetState() == ROUTE_EXECUTOR_DONE)
      {
        ServoControl_SetAngle(0U, VISION_DEMO_BEAN_A_YAW_DEGREES);
        CameraTilt_SetDown();
        K230Link_InvalidateResult();
        memset(&g_display_result, 0, sizeof(g_display_result));
        (void)K230Link_SelectTask(K230_TASK_BEAN);
        g_deadline = HAL_GetTick() + VISION_DEMO_POINT_DWELL_MS;
        EnterState(VISION_ROUTE_DEMO_SCAN_BEAN_A);
      }
      else if (RouteExecutor_GetState() == ROUTE_EXECUTOR_FAULT) EnterFault();
      break;

    case VISION_ROUTE_DEMO_SCAN_BEAN_A:
      AcceptLatestFrame(K230_TASK_BEAN);
      if (DeadlineExpired())
      {
        ServoControl_SetAngle(0U, VISION_DEMO_BEAN_B_YAW_DEGREES);
        g_deadline = HAL_GetTick() + VISION_DEMO_POINT_DWELL_MS;
        EnterState(VISION_ROUTE_DEMO_SCAN_BEAN_B);
      }
      break;

    case VISION_ROUTE_DEMO_SCAN_BEAN_B:
      AcceptLatestFrame(K230_TASK_BEAN);
      if (DeadlineExpired()) EnterState(VISION_ROUTE_DEMO_COMPLETE);
      break;

    default:
      break;
  }
}

VisionRouteDemoState VisionRouteDemo_GetState(void)
{
  return g_state;
}

uint8_t VisionRouteDemo_IsRunning(void)
{
  return (g_state >= VISION_ROUTE_DEMO_LIFT_Z) &&
         (g_state <= VISION_ROUTE_DEMO_SCAN_BEAN_B);
}

uint8_t VisionRouteDemo_GetDisplayResult(K230VisionResult *result)
{
  if ((result == NULL) || !g_display_result.valid) return 0U;
  *result = g_display_result;
  return 1U;
}

uint8_t VisionRouteDemo_GetResultWarning(void)
{
  return g_result_warning;
}
