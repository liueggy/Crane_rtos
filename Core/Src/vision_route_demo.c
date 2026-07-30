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
#define VISION_DEMO_RESCAN_DWELL_MS         2000U
#define VISION_DEMO_DIRECTION_SETTLE_MS     300U
#define VISION_DEMO_NUMBER_YAW_DEGREES         0U
#define VISION_DEMO_NUMBER_SIDE_YAW_DEGREES   54U
#define VISION_DEMO_BEAN_A_YAW_DEGREES       180U
#define VISION_DEMO_BEAN_B_YAW_DEGREES       194U
#define VISION_DEMO_DISPLAY_WIDTH             640U
#define VISION_DEMO_STABLE_HITS                  2U
#define VISION_DEMO_ROW_CENTER_X                320U
#define VISION_DEMO_ROW_ROI_HALF_WIDTH          224U
#define VISION_DEMO_ROW_SLOT_WINDOW_PULSES     4000U
#define VISION_DEMO_ROW_STABLE_SCORE            220U
#define VISION_DEMO_ROW_SCORE_MARGIN             50U
/* 侧视姿态下画面左侧目标对应物理5号箱；安装方向变化时只改此常量。 */
#define VISION_DEMO_SIDE_LEFT_IS_NUMBER_5         1U
#define VISION_NUMBER_COMPLETE_MASK             0x1FU
#define VISION_BEAN_COMPLETE_MASK               0x07U

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
static uint8_t g_retry_active;
static uint8_t g_number_retry_used;
static uint8_t g_bean_retry_used;
static uint8_t g_rescan_cursor;
static uint32_t g_deadline;
static uint32_t g_not_before_tick;
static uint16_t g_last_sequence;
static K230VisionResult g_number_display_result;
static K230VisionResult g_bean_display_result;
static VisionSlotVote g_number_votes[5];
static VisionSlotVote g_bean_votes[3];
static uint16_t g_number_row_scores[5][5];
static uint8_t g_number_row_support[5][5];
static uint8_t g_number_row_peak_confidence[5][5];
static VisionSurveyMap g_survey_map;

static void AcceptDynamicNumberRowFrame(const K230VisionResult *result);
static void AcceptNumberSideFrame(const K230VisionResult *result);
static void AcceptBeanPoseFrame(const K230VisionResult *result, uint8_t slot);
static void RefreshSurveyMap(void);
static uint8_t NumberMapValid(void);
static uint8_t BeanMapValid(void);

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
  WorldMap_InvalidateAll();
  StopMotion();
  EnterState(VISION_ROUTE_DEMO_FAULT);
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
  K230VisionResult *display = (task == K230_TASK_NUMBER) ?
                              &g_number_display_result :
                              &g_bean_display_result;
  memset(display, 0, sizeof(*display));
  display->valid = 1U;
  display->task = (uint8_t)task;
  display->sequence = g_last_sequence;
  display->received_tick_ms = HAL_GetTick();
  for (uint8_t slot = 0U; slot < slot_count; ++slot)
  {
    if (!votes[slot].stable) continue;
    K230VisionTarget *target = &display->targets[display->count++];
    target->semantic = votes[slot].candidate;
    target->confidence_percent = votes[slot].confidence;
    target->center_x = (uint16_t)(((uint32_t)(slot * 2U + 1U) *
                                  VISION_DEMO_DISPLAY_WIDTH) /
                                 (slot_count * 2U));
  }
}

static void AcceptVote(VisionSlotVote *vote, uint8_t semantic,
                       uint8_t confidence)
{
  if (vote->stable && (vote->candidate == semantic)) return;
  if (vote->candidate == semantic)
  {
    if (vote->hits < UINT8_MAX) ++vote->hits;
    if (confidence > vote->confidence) vote->confidence = confidence;
  }
  else if (!vote->stable &&
           ((vote->hits < VISION_DEMO_STABLE_HITS) ||
            (confidence > vote->confidence)))
  {
    vote->candidate = semantic;
    vote->hits = 1U;
    vote->confidence = confidence;
  }
  if (vote->hits >= VISION_DEMO_STABLE_HITS) vote->stable = 1U;
}

static void AcceptLatestFrame(K230VisionTask task)
{
  K230VisionResult result;

  if (!K230Link_GetLatestResult(&result) || !result.valid ||
      (result.task != (uint8_t)task) ||
      (result.sequence == g_last_sequence) ||
      !K230Link_IsResultFresh(1000U)) return;

  g_last_sequence = result.sequence;
  if ((task == K230_TASK_NUMBER) &&
      ((g_state == VISION_ROUTE_DEMO_SCAN_NUMBER_START) ||
       (g_state == VISION_ROUTE_DEMO_SCAN_NUMBER_ROW) ||
       (g_state == VISION_ROUTE_DEMO_RESCAN_NUMBER_DWELL)))
  {
    AcceptDynamicNumberRowFrame(&result);
    RebuildDisplayResult(K230_TASK_NUMBER, g_number_votes, 5U);
    RefreshSurveyMap();
    return;
  }
  if ((task == K230_TASK_NUMBER) &&
      ((g_state == VISION_ROUTE_DEMO_SCAN_NUMBER_SIDE) ||
       (g_state == VISION_ROUTE_DEMO_RESCAN_NUMBER_SIDE)))
  {
    AcceptNumberSideFrame(&result);
    RebuildDisplayResult(K230_TASK_NUMBER, g_number_votes, 5U);
    RefreshSurveyMap();
    return;
  }
  if (task == K230_TASK_BEAN)
  {
    uint8_t slot = (g_state == VISION_ROUTE_DEMO_SCAN_BEAN_A) ? 0U : 1U;
    AcceptBeanPoseFrame(&result, slot);
    RebuildDisplayResult(K230_TASK_BEAN, g_bean_votes, 3U);
    RefreshSurveyMap();
  }
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

static void ResetResults(void)
{
  memset(&g_number_display_result, 0, sizeof(g_number_display_result));
  memset(&g_bean_display_result, 0, sizeof(g_bean_display_result));
  memset(g_number_votes, 0, sizeof(g_number_votes));
  memset(g_bean_votes, 0, sizeof(g_bean_votes));
  memset(g_number_row_scores, 0, sizeof(g_number_row_scores));
  memset(g_number_row_support, 0, sizeof(g_number_row_support));
  memset(g_number_row_peak_confidence, 0,
         sizeof(g_number_row_peak_confidence));
  memset(&g_survey_map, 0, sizeof(g_survey_map));
  g_result_warning = 0U;
  g_retry_active = 0U;
  g_number_retry_used = 0U;
  g_bean_retry_used = 0U;
  g_rescan_cursor = 1U;
  g_last_sequence = 0U;
}

void VisionRouteDemo_Init(void)
{
  ResetResults();
  EnterState(VISION_ROUTE_DEMO_IDLE);
}

void VisionRouteDemo_Abort(void)
{
  if (VisionRouteDemo_IsRunning()) WorldMap_InvalidateAll();
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

  (void)VisionRouteDemo_StartCompetition();
}

uint8_t VisionRouteDemo_StartCompetition(void)
{
  if (VisionRouteDemo_IsRunning() || HasSafetyFault()) return 0U;
  StopMotion();
  ResetResults();
  ServoControl_SetAngle(0U, VISION_DEMO_NUMBER_YAW_DEGREES);
  CameraTilt_SetLevel();
  EnterState(VISION_ROUTE_DEMO_LIFT_Z);
  return 1U;
}

static int32_t NumberRowSlotX(uint8_t physical_slot)
{
  static const int32_t x[3] = {
    NUMBER_SLOT_BOTTOM_RIGHT_X_PULSES,
    NUMBER_SLOT_BOTTOM_CENTER_X_PULSES,
    NUMBER_SLOT_BOTTOM_LEFT_X_PULSES
  };
  return ((physical_slot >= 1U) && (physical_slot <= 3U)) ?
         x[physical_slot - 1U] : WORLD_MAP_UNCALIBRATED;
}

static uint8_t StartBeanRoute(void)
{
  ServoControl_SetAngle(0U, VISION_DEMO_BEAN_A_YAW_DEGREES);
  if (!RouteExecutor_Start(ROBOT_ROUTE_SURVEY_NUMBER_TO_BEAN_DIRECT)) return 0U;
  g_retry_active = 0U;
  EnterState(VISION_ROUTE_DEMO_MOVE_BEAN);
  return 1U;
}

static void ContinueNumberRetry(void)
{
  while ((g_rescan_cursor <= 3U) &&
         g_number_votes[g_rescan_cursor].stable) ++g_rescan_cursor;
  if (g_rescan_cursor <= 3U)
  {
    EnterState(VISION_ROUTE_DEMO_RESCAN_NUMBER_MOVE);
    return;
  }
  if (!g_number_votes[0].stable || !g_number_votes[4].stable)
  {
    ServoControl_SetAngle(0U, VISION_DEMO_NUMBER_SIDE_YAW_DEGREES);
    g_deadline = HAL_GetTick() + VISION_DEMO_RESCAN_DWELL_MS;
    EnterState(VISION_ROUTE_DEMO_RESCAN_NUMBER_SIDE);
    return;
  }
  RefreshSurveyMap();
  if (!NumberMapValid() || !StartBeanRoute()) EnterFault();
}

static void BeginNumberRetry(void)
{
  if (g_number_retry_used)
  {
    EnterFault();
    return;
  }
  g_number_retry_used = 1U;
  g_retry_active = 1U;
  g_result_warning = 1U;
  if (g_survey_map.number_valid_mask == VISION_NUMBER_COMPLETE_MASK)
  {
    /* 槽位齐全但语义重复时，整排和两侧全部重新投票。 */
    memset(g_number_votes, 0, sizeof(g_number_votes));
    memset(g_number_row_scores, 0, sizeof(g_number_row_scores));
    memset(g_number_row_support, 0, sizeof(g_number_row_support));
    memset(g_number_row_peak_confidence, 0,
           sizeof(g_number_row_peak_confidence));
  }
  g_rescan_cursor = 1U;
  ContinueNumberRetry();
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
          WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X),
              WorldMap_GetPose()->x_valid,
              StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
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
        else
        {
          WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X),
                                   1U,
                                   StepperAxis_GetPositionPulses(STEPPER_AXIS_Z),
                                   1U);
          EnterState(VISION_ROUTE_DEMO_MOVE_NUMBER);
        }
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
        RefreshSurveyMap();
        if (NumberMapValid())
        {
          if (!StartBeanRoute()) EnterFault();
        }
        else BeginNumberRetry();
      }
      break;

    case VISION_ROUTE_DEMO_RESCAN_NUMBER_MOVE:
      if (!g_stage_started)
      {
        int32_t current_x = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
        int32_t target_x = NumberRowSlotX(g_rescan_cursor);
        uint32_t pulses = (uint32_t)((current_x > target_x) ?
                          current_x - target_x : target_x - current_x);
        ServoControl_SetAngle(0U, VISION_DEMO_NUMBER_YAW_DEGREES);
        if (pulses == 0U)
        {
          g_deadline = HAL_GetTick() + VISION_DEMO_RESCAN_DWELL_MS;
          EnterState(VISION_ROUTE_DEMO_RESCAN_NUMBER_DWELL);
        }
        else if (StepperAxis_MovePulses(STEPPER_AXIS_X, pulses,
                 (current_x > target_x) ? 1U : 0U) != HAL_OK) EnterFault();
        else
        {
          g_stage_started = 1U;
          g_deadline = HAL_GetTick() + VISION_DEMO_X_MOVE_TIMEOUT_MS;
        }
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X))
      {
        if ((StepperAxis_GetRemainingPulses(STEPPER_AXIS_X) != 0U) ||
            (StepperAxis_GetPositionPulses(STEPPER_AXIS_X) !=
             NumberRowSlotX(g_rescan_cursor))) EnterFault();
        else
        {
          g_deadline = HAL_GetTick() + VISION_DEMO_RESCAN_DWELL_MS;
          EnterState(VISION_ROUTE_DEMO_RESCAN_NUMBER_DWELL);
        }
      }
      else if (DeadlineExpired()) EnterFault();
      break;

    case VISION_ROUTE_DEMO_RESCAN_NUMBER_DWELL:
      AcceptLatestFrame(K230_TASK_NUMBER);
      if (DeadlineExpired())
      {
        ++g_rescan_cursor;
        ContinueNumberRetry();
      }
      break;

    case VISION_ROUTE_DEMO_RESCAN_NUMBER_SIDE:
      AcceptLatestFrame(K230_TASK_NUMBER);
      if (DeadlineExpired())
      {
        RefreshSurveyMap();
        if (!NumberMapValid() || !StartBeanRoute()) EnterFault();
      }
      break;

    case VISION_ROUTE_DEMO_MOVE_BEAN:
      if (RouteExecutor_GetState() == ROUTE_EXECUTOR_DONE)
      {
        ServoControl_SetAngle(0U, VISION_DEMO_BEAN_A_YAW_DEGREES);
        CameraTilt_SetDown();
        K230Link_InvalidateResult();
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
        g_deadline = HAL_GetTick() + (g_retry_active ?
                     VISION_DEMO_RESCAN_DWELL_MS : VISION_DEMO_POINT_DWELL_MS);
        EnterState(VISION_ROUTE_DEMO_SCAN_BEAN_B);
      }
      break;

    case VISION_ROUTE_DEMO_SCAN_BEAN_B:
      AcceptLatestFrame(K230_TASK_BEAN);
      if (DeadlineExpired())
      {
        RefreshSurveyMap();
        if (BeanMapValid())
        {
          g_retry_active = 0U;
          EnterState(VISION_ROUTE_DEMO_COMPLETE);
        }
        else if (!g_bean_retry_used)
        {
          g_bean_retry_used = 1U;
          g_retry_active = 1U;
          g_result_warning = 1U;
          memset(g_bean_votes, 0, sizeof(g_bean_votes));
          memset(&g_survey_map.bean_at_slot, 0,
                 sizeof(g_survey_map.bean_at_slot));
          g_survey_map.bean_valid_mask = 0U;
          ServoControl_SetAngle(0U, VISION_DEMO_BEAN_A_YAW_DEGREES);
          g_deadline = HAL_GetTick() + VISION_DEMO_RESCAN_DWELL_MS;
          EnterState(VISION_ROUTE_DEMO_SCAN_BEAN_A);
        }
        else EnterFault();
      }
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
  const K230VisionResult *saved;
  if (result == NULL) return 0U;
  saved = (result->task == K230_TASK_BEAN) ?
          &g_bean_display_result : &g_number_display_result;
  if (!saved->valid) return 0U;
  *result = *saved;
  return 1U;
}

static uint32_t AbsDifference32(int32_t a, int32_t b)
{
  int32_t difference = a - b;
  return (uint32_t)((difference < 0) ? -difference : difference);
}

static void AcceptNumberSideFrame(const K230VisionResult *result)
{
  const K230VisionTarget *left = NULL;
  const K230VisionTarget *right = NULL;
  for (uint8_t i = 0U; i < result->count; ++i)
  {
    const K230VisionTarget *target = &result->targets[i];
    if (!IsExpectedSemantic(K230_TASK_NUMBER, target->semantic)) continue;
    if ((left == NULL) || (target->center_x < left->center_x)) left = target;
    if ((right == NULL) || (target->center_x > right->center_x)) right = target;
  }
  if (left != NULL)
    AcceptVote(&g_number_votes[VISION_DEMO_SIDE_LEFT_IS_NUMBER_5 ? 4U : 0U],
               left->semantic, left->confidence_percent);
  if ((right != NULL) && (right != left))
    AcceptVote(&g_number_votes[VISION_DEMO_SIDE_LEFT_IS_NUMBER_5 ? 0U : 4U],
               right->semantic, right->confidence_percent);
}

static void AcceptBeanPoseFrame(const K230VisionResult *result, uint8_t slot)
{
  const K230VisionTarget *best = NULL;
  if (slot >= 2U) return;
  for (uint8_t i = 0U; i < result->count; ++i)
  {
    const K230VisionTarget *target = &result->targets[i];
    if (!IsExpectedSemantic(K230_TASK_BEAN, target->semantic)) continue;
    if ((best == NULL) ||
        (target->confidence_percent > best->confidence_percent)) best = target;
  }
  if (best != NULL)
    AcceptVote(&g_bean_votes[slot], best->semantic, best->confidence_percent);
}

static void RefreshSurveyMap(void)
{
  uint8_t bean_seen = 0U;
  uint8_t missing_slot = 2U;
  memset(&g_survey_map, 0, sizeof(g_survey_map));
  g_survey_map.last_sequence = g_last_sequence;
  for (uint8_t slot = 0U; slot < 5U; ++slot)
  {
    if (!g_number_votes[slot].stable) continue;
    g_survey_map.number_at_slot[slot] = g_number_votes[slot].candidate;
    g_survey_map.number_valid_mask |= (uint8_t)(1U << slot);
  }
  for (uint8_t slot = 0U; slot < 2U; ++slot)
  {
    uint8_t bean;
    if (!g_bean_votes[slot].stable) continue;
    bean = g_bean_votes[slot].candidate;
    g_survey_map.bean_at_slot[slot] = bean;
    g_survey_map.bean_valid_mask |= (uint8_t)(1U << slot);
    bean_seen |= (bean == K230_SEMANTIC_BEAN_L) ? 0x01U :
                 (bean == K230_SEMANTIC_BEAN_H) ? 0x02U :
                 (bean == K230_SEMANTIC_BEAN_B) ? 0x04U : 0U;
  }
  if ((g_survey_map.bean_valid_mask == 0x03U) &&
      ((bean_seen == 0x03U) || (bean_seen == 0x05U) || (bean_seen == 0x06U)))
  {
    uint8_t missing = (bean_seen == 0x03U) ? K230_SEMANTIC_BEAN_B :
                      (bean_seen == 0x05U) ? K230_SEMANTIC_BEAN_H :
                                             K230_SEMANTIC_BEAN_L;
    g_survey_map.bean_at_slot[missing_slot] = missing;
    g_survey_map.bean_valid_mask |= (uint8_t)(1U << missing_slot);
    g_bean_votes[missing_slot].candidate = missing;
    g_bean_votes[missing_slot].hits = VISION_DEMO_STABLE_HITS;
    g_bean_votes[missing_slot].confidence = 100U;
    g_bean_votes[missing_slot].stable = 1U;
    RebuildDisplayResult(K230_TASK_BEAN, g_bean_votes, 3U);
  }
}

static uint8_t NumberMapValid(void)
{
  uint8_t seen = 0U;
  if (g_survey_map.number_valid_mask != VISION_NUMBER_COMPLETE_MASK) return 0U;
  for (uint8_t slot = 0U; slot < 5U; ++slot)
  {
    uint8_t number = g_survey_map.number_at_slot[slot];
    if ((number < 1U) || (number > 5U) ||
        (seen & (uint8_t)(1U << (number - 1U)))) return 0U;
    seen |= (uint8_t)(1U << (number - 1U));
  }
  return seen == VISION_NUMBER_COMPLETE_MASK;
}

static uint8_t BeanMapValid(void)
{
  uint8_t seen = 0U;
  if (g_survey_map.bean_valid_mask != VISION_BEAN_COMPLETE_MASK) return 0U;
  for (uint8_t slot = 0U; slot < 3U; ++slot)
  {
    uint8_t bean = g_survey_map.bean_at_slot[slot];
    uint8_t bit = (bean == K230_SEMANTIC_BEAN_L) ? 0x01U :
                  (bean == K230_SEMANTIC_BEAN_H) ? 0x02U :
                  (bean == K230_SEMANTIC_BEAN_B) ? 0x04U : 0U;
    if ((bit == 0U) || (seen & bit)) return 0U;
    seen |= bit;
  }
  return seen == VISION_BEAN_COMPLETE_MASK;
}

/* 横排扫描不再按单帧画面五等分。夹爪移动时先用龙门绝对X选中最近的
 * 物理2/3/4号箱，再从画面中央ROI选择质量最高的数字框做置信度累计。 */
static void AcceptDynamicNumberRowFrame(const K230VisionResult *result)
{
  static const int32_t row_x[3] = {
    NUMBER_SLOT_BOTTOM_RIGHT_X_PULSES,
    NUMBER_SLOT_BOTTOM_CENTER_X_PULSES,
    NUMBER_SLOT_BOTTOM_LEFT_X_PULSES
  };
  static const uint8_t physical_slot[3] = {1U, 2U, 3U};
  int32_t gantry_x = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
  uint8_t nearest = 0U;
  uint32_t nearest_distance = AbsDifference32(gantry_x, row_x[0]);
  const K230VisionTarget *best_target = NULL;
  uint16_t best_quality = 0U;

  for (uint8_t i = 1U; i < 3U; ++i)
  {
    uint32_t distance = AbsDifference32(gantry_x, row_x[i]);
    if (distance < nearest_distance)
    {
      nearest = i;
      nearest_distance = distance;
    }
  }
  if (nearest_distance > VISION_DEMO_ROW_SLOT_WINDOW_PULSES) return;

  for (uint8_t i = 0U; i < result->count; ++i)
  {
    const K230VisionTarget *target = &result->targets[i];
    uint32_t center_error;
    uint16_t quality;
    if (!IsExpectedSemantic(K230_TASK_NUMBER, target->semantic)) continue;
    center_error = (target->center_x > VISION_DEMO_ROW_CENTER_X) ?
        (target->center_x - VISION_DEMO_ROW_CENTER_X) :
        (VISION_DEMO_ROW_CENTER_X - target->center_x);
    if (center_error > VISION_DEMO_ROW_ROI_HALF_WIDTH) continue;
    quality = (uint16_t)(target->confidence_percent +
        (VISION_DEMO_ROW_ROI_HALF_WIDTH - center_error) * 100U /
        VISION_DEMO_ROW_ROI_HALF_WIDTH);
    if ((best_target == NULL) || (quality > best_quality))
    {
      best_target = target;
      best_quality = quality;
    }
  }
  if (best_target == NULL) return;

  {
    uint8_t slot = physical_slot[nearest];
    uint8_t semantic_index = (uint8_t)(best_target->semantic - 1U);
    uint16_t best_score = 0U;
    uint16_t second_score = 0U;
    uint8_t best_semantic = 0U;
    uint32_t accumulated = (uint32_t)g_number_row_scores[slot][semantic_index] +
                           best_quality;
    g_number_row_scores[slot][semantic_index] =
        (accumulated > UINT16_MAX) ? UINT16_MAX : (uint16_t)accumulated;
    if (g_number_row_support[slot][semantic_index] < UINT8_MAX)
      ++g_number_row_support[slot][semantic_index];
    if (best_target->confidence_percent >
        g_number_row_peak_confidence[slot][semantic_index])
      g_number_row_peak_confidence[slot][semantic_index] =
          best_target->confidence_percent;

    for (uint8_t semantic = 0U; semantic < 5U; ++semantic)
    {
      uint16_t score = g_number_row_scores[slot][semantic];
      if (score > best_score)
      {
        second_score = best_score;
        best_score = score;
        best_semantic = semantic;
      }
      else if (score > second_score) second_score = score;
    }
    if ((g_number_row_support[slot][best_semantic] >= 2U) &&
        (best_score >= VISION_DEMO_ROW_STABLE_SCORE) &&
        (best_score >= (uint16_t)(second_score + VISION_DEMO_ROW_SCORE_MARGIN)))
    {
      g_number_votes[slot].candidate = (uint8_t)(best_semantic + 1U);
      g_number_votes[slot].hits = g_number_row_support[slot][best_semantic];
      g_number_votes[slot].confidence =
          g_number_row_peak_confidence[slot][best_semantic];
      g_number_votes[slot].stable = 1U;
    }
  }
}

uint8_t VisionRouteDemo_GetResultWarning(void)
{
  return g_result_warning;
}

uint8_t VisionRouteDemo_IsComplete(void)
{
  return (g_state == VISION_ROUTE_DEMO_COMPLETE) &&
         NumberMapValid() && BeanMapValid();
}

uint8_t VisionRouteDemo_IsRetrying(void)
{
  return g_retry_active;
}

const VisionSurveyMap *VisionRouteDemo_GetSurveyMap(void)
{
  return &g_survey_map;
}
