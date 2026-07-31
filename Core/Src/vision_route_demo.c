#include "vision_route_demo.h"

#include "app_config.h"
#include "camera_tilt.h"
#include "chassis_motion.h"
#include "initialization_debug.h"
#include "mission_action.h"
#include "mission_navigator.h"
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
#define VISION_DEMO_X_MOVE_TIMEOUT_MS     18000U
#define VISION_DEMO_POINT_DWELL_MS          1000U
#define VISION_DEMO_POINT_SETTLE_MS          400U
#define VISION_DEMO_RESCAN_DWELL_MS         2000U
#define VISION_DEMO_TASK_SWITCH_TIMEOUT_MS  2000U
#define VISION_DEMO_DIRECTION_SETTLE_MS     300U
#define VISION_DEMO_NUMBER_YAW_DEGREES         0U
#define VISION_DEMO_NUMBER_SIDE_YAW_DEGREES   54U
#define VISION_DEMO_BEAN_A_YAW_DEGREES       180U
#define VISION_DEMO_BEAN_B_YAW_DEGREES       194U
#define VISION_DEMO_DISPLAY_WIDTH             640U
#define VISION_DEMO_STABLE_HITS                  2U
#define VISION_DEMO_ROW_SLOT_WINDOW_PULSES     4000U
#define VISION_NUMBER_COMPLETE_MASK             0x1FU
#define VISION_BEAN_COMPLETE_MASK               0x07U

typedef struct
{
  uint8_t candidate;
  uint8_t hits;
  uint8_t stable;
  uint8_t confidence;
} VisionSlotVote;

typedef enum
{
  DEBUG_STREAM_WAIT_TASK = 0,
  DEBUG_STREAM_WAIT_SETTLE,
  DEBUG_STREAM_ACQUIRE,
  DEBUG_STREAM_DONE
} DebugStreamPhase;

static VisionRouteDemoState g_state;
static uint8_t g_stage_started;
static uint8_t g_result_warning;
static uint8_t g_debug_step_scan;
static uint8_t g_competition_mode;
static uint8_t g_retry_active;
static uint8_t g_number_retry_used;
static uint8_t g_bean_retry_used;
static uint8_t g_rescan_cursor;
static uint8_t g_debug_row_slot;
static DebugStreamPhase g_debug_stream_phase;
static uint16_t g_debug_stream_settle_ms;
static uint16_t g_debug_stream_dwell_ms;
static uint8_t g_debug_stream_majority_slot;
static uint32_t g_debug_stream_switch_deadline;
static uint32_t g_deadline;
static uint32_t g_not_before_tick;
static uint16_t g_last_sequence;
static K230VisionResult g_number_display_result;
static K230VisionResult g_bean_display_result;
static VisionSlotVote g_number_votes[5];
static VisionSlotVote g_bean_votes[3];
/* 调试页定点采集只统计K230串口返回的数字次数，不参与连续扫描加权。 */
static uint16_t g_debug_number_counts[5][5];
/* 豆子调试采集同样按A/B姿态分别统计L/H/B出现次数。 */
static uint16_t g_debug_bean_counts[2][3];
static VisionSurveyMap g_survey_map;

static void AcceptDynamicNumberRowFrame(const K230VisionResult *result);
static void AcceptDebugNumberFrame(uint8_t physical_slot);
static void AcceptDebugBeanFrame(uint8_t physical_slot);
static void AcceptNumberSideFrame(const K230VisionResult *result);
static void AcceptBeanPoseFrame(const K230VisionResult *result, uint8_t slot);
static void RefreshSurveyMap(void);
static uint8_t NumberSemanticUsedByOtherSlot(uint8_t semantic,
                                             uint8_t physical_slot);
static uint8_t NumberMapValid(void);
static uint8_t BeanMapValid(void);
static void BeginNumberRetry(void);

static void InvalidateDuplicateVotes(VisionSlotVote *votes, uint8_t count)
{
  for (uint8_t left = 0U; left < count; ++left)
  {
    if (!votes[left].stable) continue;
    for (uint8_t right = (uint8_t)(left + 1U); right < count; ++right)
    {
      if (!votes[right].stable ||
          (votes[left].candidate != votes[right].candidate)) continue;
      if (votes[left].hits > votes[right].hits) votes[right].stable = 0U;
      else if (votes[right].hits > votes[left].hits) votes[left].stable = 0U;
      else
      {
        votes[left].stable = 0U;
        votes[right].stable = 0U;
      }
    }
  }
}

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
  MissionNavigator_Abort();
  MissionAction_Abort();
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

/* 0失败，1已启动/仍在运行，2已经位于目标。 */
static uint8_t MoveAxisTo(StepperAxisId axis, int32_t target)
{
  int32_t current = StepperAxis_GetPositionPulses(axis);
  uint32_t pulses;
  if (StepperAxis_IsPulseMoveActive(axis)) return 1U;
  if (current == target) return 2U;
  if ((target < 0) ||
      ((axis == STEPPER_AXIS_X) &&
       (target > (int32_t)STEPPER_X_TRAVEL_PULSES)) ||
      ((axis == STEPPER_AXIS_Z) &&
       (target > (int32_t)STEPPER_Z_TRAVEL_PULSES))) return 0U;
  pulses = (uint32_t)((current > target) ? current - target : target - current);
  return (StepperAxis_MovePulses(axis, pulses,
          (current > target) ? 1U : 0U) == HAL_OK) ? 1U : 0U;
}

static uint8_t AxisReached(StepperAxisId axis, int32_t target)
{
  return !StepperAxis_IsPulseMoveActive(axis) &&
         (StepperAxis_GetRemainingPulses(axis) == 0U) &&
         (StepperAxis_GetPositionPulses(axis) == target);
}

static void FinishBeanScan(void)
{
  g_retry_active = 0U;
  if (g_competition_mode) EnterState(VISION_ROUTE_DEMO_COMPLETE);
  else EnterState(VISION_ROUTE_DEMO_POST_PREPARE);
}

static uint8_t NumberSemanticUsedByOtherSlot(uint8_t semantic,
                                             uint8_t physical_slot)
{
  /* 物理1号是排除结果，不参与2/3/4/5直接采集时的互斥占用。 */
  for (uint8_t slot = 1U; slot < 5U; ++slot)
  {
    if ((slot != physical_slot) && g_number_votes[slot].stable &&
        (g_number_votes[slot].candidate == semantic)) return 1U;
  }
  return 0U;
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
    RefreshSurveyMap();
    RebuildDisplayResult(K230_TASK_NUMBER, g_number_votes, 5U);
    return;
  }
  if ((task == K230_TASK_NUMBER) &&
      ((g_state == VISION_ROUTE_DEMO_SCAN_NUMBER_SIDE) ||
       (g_state == VISION_ROUTE_DEMO_RESCAN_NUMBER_SIDE)))
  {
    AcceptNumberSideFrame(&result);
    RefreshSurveyMap();
    RebuildDisplayResult(K230_TASK_NUMBER, g_number_votes, 5U);
    return;
  }
  if (task == K230_TASK_BEAN)
  {
    uint8_t slot = (g_state == VISION_ROUTE_DEMO_SCAN_BEAN_A) ? 0U : 1U;
    AcceptBeanPoseFrame(&result, slot);
    RefreshSurveyMap();
    RebuildDisplayResult(K230_TASK_BEAN, g_bean_votes, 3U);
  }
}

static void ResetDebugNumberSlot(uint8_t physical_slot)
{
  if (physical_slot >= 5U) return;
  memset(g_debug_number_counts[physical_slot], 0,
         sizeof(g_debug_number_counts[physical_slot]));
  memset(&g_number_votes[physical_slot], 0,
         sizeof(g_number_votes[physical_slot]));
  RefreshSurveyMap();
  RebuildDisplayResult(K230_TASK_NUMBER, g_number_votes, 5U);
}

static void ResetDebugBeanSlot(uint8_t physical_slot)
{
  if (physical_slot >= 2U) return;
  memset(g_debug_bean_counts[physical_slot], 0,
         sizeof(g_debug_bean_counts[physical_slot]));
  memset(&g_bean_votes[physical_slot], 0,
         sizeof(g_bean_votes[physical_slot]));
  RefreshSurveyMap();
  RebuildDisplayResult(K230_TASK_BEAN, g_bean_votes, 3U);
}

static void BeginDebugStreamWindow(uint16_t settle_ms, uint16_t dwell_ms,
                                   uint8_t majority_slot)
{
  uint32_t now = HAL_GetTick();
  K230Link_InvalidateResult();
  g_debug_stream_phase = DEBUG_STREAM_WAIT_TASK;
  g_debug_stream_settle_ms = settle_ms;
  g_debug_stream_dwell_ms = dwell_ms;
  g_debug_stream_majority_slot = majority_slot;
  g_debug_stream_switch_deadline =
      now + VISION_DEMO_TASK_SWITCH_TIMEOUT_MS;
}

/* K230一直自动发送结果；STM32只在本地时间窗内统计新字符。 */
static uint8_t ProcessDebugStreamWindow(K230VisionTask task)
{
  uint32_t now = HAL_GetTick();
  if (g_debug_stream_phase == DEBUG_STREAM_WAIT_TASK)
  {
    K230TaskSwitchState switch_state = K230Link_GetTaskSwitchState();
    if (switch_state == K230_TASK_SWITCH_FAILED)
    {
      if (!g_competition_mode) return 0U;
      g_result_warning = 1U;
      g_debug_stream_phase = DEBUG_STREAM_DONE;
      return 2U;
    }
    if ((switch_state != K230_TASK_SWITCH_IDLE) ||
        (K230Link_GetSelectedTask() != task))
    {
      if (!TimeReached(g_debug_stream_switch_deadline)) return 1U;
      if (!g_competition_mode) return 0U;
      g_result_warning = 1U;
      g_debug_stream_phase = DEBUG_STREAM_DONE;
      return 2U;
    }
    g_debug_stream_phase = DEBUG_STREAM_WAIT_SETTLE;
    g_not_before_tick = now + g_debug_stream_settle_ms;
    return 1U;
  }
  if (g_debug_stream_phase == DEBUG_STREAM_WAIT_SETTLE)
  {
    if (!TimeReached(g_not_before_tick)) return 1U;
    K230Link_InvalidateResult();
    if ((task == K230_TASK_NUMBER) &&
        (g_debug_stream_majority_slot < 5U))
    {
      if (!(g_competition_mode && g_retry_active))
        ResetDebugNumberSlot(g_debug_stream_majority_slot);
    }
    else if (task == K230_TASK_BEAN)
    {
      if (!(g_competition_mode && g_retry_active))
        ResetDebugBeanSlot((g_state == VISION_ROUTE_DEMO_SCAN_BEAN_A) ? 0U : 1U);
    }
    g_debug_stream_phase = DEBUG_STREAM_ACQUIRE;
    g_deadline = now + g_debug_stream_dwell_ms;
    return 1U;
  }
  if (g_debug_stream_phase == DEBUG_STREAM_ACQUIRE)
  {
    if ((task == K230_TASK_NUMBER) &&
        (g_debug_stream_majority_slot < 5U))
      AcceptDebugNumberFrame(g_debug_stream_majority_slot);
    else if (task == K230_TASK_BEAN)
      AcceptDebugBeanFrame((g_state == VISION_ROUTE_DEMO_SCAN_BEAN_A) ? 0U : 1U);
    else AcceptLatestFrame(task);
    if (DeadlineExpired())
    {
      K230Link_InvalidateResult();
      g_debug_stream_phase = DEBUG_STREAM_DONE;
      return 2U;
    }
    return 1U;
  }
  return 2U;
}

/* 定点多数票：当前物理框每收到一个新字符立即刷新领先结果。
 * 已被其他物理2/3/4/5号框锁定的数字不再参与当前框竞争；平票时保留当前
 * 领先值避免OLED框反复变空。采集窗口结束后该值保持锁存。 */
static void AcceptDebugNumberFrame(uint8_t physical_slot)
{
  K230VisionResult result;
  uint16_t best_count = 0U;
  uint8_t best_semantic = UINT8_MAX;
  uint8_t current_semantic;

  if ((physical_slot >= 5U) || !K230Link_GetLatestResult(&result) ||
      !result.valid || (result.task != K230_TASK_NUMBER) ||
      (result.sequence == g_last_sequence) ||
      !K230Link_IsResultFresh(1000U)) return;
  g_last_sequence = result.sequence;

  for (uint8_t i = 0U; i < result.count; ++i)
  {
    const K230VisionTarget *target = &result.targets[i];
    uint8_t semantic_index;
    if (!IsExpectedSemantic(K230_TASK_NUMBER, target->semantic)) continue;
    semantic_index = (uint8_t)(target->semantic - K230_SEMANTIC_NUMBER_1);
    if (g_debug_number_counts[physical_slot][semantic_index] < UINT16_MAX)
      ++g_debug_number_counts[physical_slot][semantic_index];
  }

  current_semantic = (g_number_votes[physical_slot].candidate >=
                      K230_SEMANTIC_NUMBER_1) ?
                     (uint8_t)(g_number_votes[physical_slot].candidate -
                               K230_SEMANTIC_NUMBER_1) : UINT8_MAX;
  for (uint8_t semantic = 0U; semantic < 5U; ++semantic)
  {
    uint16_t count = g_debug_number_counts[physical_slot][semantic];
    uint8_t value = (uint8_t)(K230_SEMANTIC_NUMBER_1 + semantic);
    if (NumberSemanticUsedByOtherSlot(value, physical_slot)) continue;
    if (count > best_count)
    {
      best_count = count;
      best_semantic = semantic;
    }
  }
  /* 平票时优先保留上一次已显示的领先值。 */
  if ((current_semantic < 5U) &&
      !NumberSemanticUsedByOtherSlot(
          (uint8_t)(K230_SEMANTIC_NUMBER_1 + current_semantic), physical_slot) &&
      (g_debug_number_counts[physical_slot][current_semantic] == best_count))
    best_semantic = current_semantic;

  if ((best_count != 0U) && (best_semantic < 5U))
  {
    g_number_votes[physical_slot].candidate =
        (uint8_t)(K230_SEMANTIC_NUMBER_1 + best_semantic);
    g_number_votes[physical_slot].hits =
        (best_count > UINT8_MAX) ? UINT8_MAX : (uint8_t)best_count;
    g_number_votes[physical_slot].confidence = 100U;
    g_number_votes[physical_slot].stable = 1U;
  }
  RefreshSurveyMap();
  RebuildDisplayResult(K230_TASK_NUMBER, g_number_votes, 5U);
}

static void AcceptDebugBeanFrame(uint8_t physical_slot)
{
  K230VisionResult result;
  uint16_t best_count = 0U;
  uint8_t best_index = UINT8_MAX;
  uint8_t current_index = UINT8_MAX;

  if ((physical_slot >= 2U) || !K230Link_GetLatestResult(&result) ||
      !result.valid || (result.task != K230_TASK_BEAN) ||
      (result.sequence == g_last_sequence) ||
      !K230Link_IsResultFresh(1000U)) return;
  g_last_sequence = result.sequence;

  for (uint8_t i = 0U; i < result.count; ++i)
  {
    uint8_t semantic = result.targets[i].semantic;
    uint8_t index = (semantic == K230_SEMANTIC_BEAN_L) ? 0U :
                    (semantic == K230_SEMANTIC_BEAN_H) ? 1U :
                    (semantic == K230_SEMANTIC_BEAN_B) ? 2U : UINT8_MAX;
    if ((index < 3U) &&
        (g_debug_bean_counts[physical_slot][index] < UINT16_MAX))
      ++g_debug_bean_counts[physical_slot][index];
  }

  current_index = (g_bean_votes[physical_slot].candidate ==
                   K230_SEMANTIC_BEAN_L) ? 0U :
                  (g_bean_votes[physical_slot].candidate ==
                   K230_SEMANTIC_BEAN_H) ? 1U :
                  (g_bean_votes[physical_slot].candidate ==
                   K230_SEMANTIC_BEAN_B) ? 2U : UINT8_MAX;
  for (uint8_t index = 0U; index < 3U; ++index)
  {
    uint16_t count = g_debug_bean_counts[physical_slot][index];
    if (count > best_count)
    {
      best_count = count;
      best_index = index;
    }
  }
  if ((current_index < 3U) &&
      (g_debug_bean_counts[physical_slot][current_index] == best_count))
    best_index = current_index;

  if ((best_count != 0U) && (best_index < 3U))
  {
    static const uint8_t semantics[3] = {
      K230_SEMANTIC_BEAN_L, K230_SEMANTIC_BEAN_H, K230_SEMANTIC_BEAN_B
    };
    g_bean_votes[physical_slot].candidate = semantics[best_index];
    g_bean_votes[physical_slot].hits =
        (best_count > UINT8_MAX) ? UINT8_MAX : (uint8_t)best_count;
    g_bean_votes[physical_slot].confidence = 100U;
    g_bean_votes[physical_slot].stable = 1U;
  }
  RefreshSurveyMap();
  RebuildDisplayResult(K230_TASK_BEAN, g_bean_votes, 3U);
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
  memset(g_debug_number_counts, 0, sizeof(g_debug_number_counts));
  memset(g_debug_bean_counts, 0, sizeof(g_debug_bean_counts));
  memset(&g_survey_map, 0, sizeof(g_survey_map));
  g_result_warning = 0U;
  g_retry_active = 0U;
  g_competition_mode = 0U;
  g_number_retry_used = 0U;
  g_bean_retry_used = 0U;
  g_rescan_cursor = 1U;
  g_debug_row_slot = 1U;
  g_debug_stream_phase = DEBUG_STREAM_DONE;
  g_debug_stream_settle_ms = 0U;
  g_debug_stream_dwell_ms = 0U;
  g_debug_stream_majority_slot = UINT8_MAX;
  g_debug_stream_switch_deadline = 0U;
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

  (void)VisionRouteDemo_StartDebug();
}

static uint8_t StartRoute(uint8_t debug_step_scan, uint8_t competition_mode)
{
  if (VisionRouteDemo_IsRunning() || HasSafetyFault()) return 0U;
  StopMotion();
  ResetResults();
  g_debug_step_scan = debug_step_scan ? 1U : 0U;
  g_competition_mode = competition_mode ? 1U : 0U;
  ServoControl_SetAngle(0U, VISION_DEMO_NUMBER_YAW_DEGREES);
  CameraTilt_SetLevel();
  EnterState(VISION_ROUTE_DEMO_LIFT_Z);
  return 1U;
}

uint8_t VisionRouteDemo_StartDebug(void)
{
  return StartRoute(1U, 0U);
}

uint8_t VisionRouteDemo_StartCompetition(void)
{
  /* 正式任务与视觉Demo使用同一条已验证链路，避免两套识别策略漂移。 */
  return StartRoute(1U, 1U);
}

uint8_t VisionRouteDemo_StartBeanRescan(void)
{
  const WorldPose *pose = WorldMap_GetPose();
  if (VisionRouteDemo_IsRunning() || HasSafetyFault() || !pose->y_valid ||
      (pose->station != WORLD_STATION_UPPER_OBSTACLE_BEAN_SCAN) ||
      (StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) != 0)) return 0U;
  g_competition_mode = 1U;
  g_debug_step_scan = 1U;
  g_retry_active = 1U;
  g_result_warning = 1U;
  /* 本次比赛期只补扫一轮；失败后以部分结果结束，不无限循环。 */
  g_bean_retry_used = 2U;
  ServoControl_SetAngle(0U, VISION_DEMO_BEAN_A_YAW_DEGREES);
  CameraTilt_SetDown();
  K230Link_InvalidateResult();
  (void)K230Link_SelectTask(K230_TASK_BEAN);
  BeginDebugStreamWindow(VISION_DEMO_POINT_SETTLE_MS,
                         VISION_DEMO_RESCAN_DWELL_MS, UINT8_MAX);
  EnterState(VISION_ROUTE_DEMO_SCAN_BEAN_A);
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
    if (g_debug_step_scan)
      BeginDebugStreamWindow(VISION_DEMO_POINT_SETTLE_MS,
                             VISION_DEMO_RESCAN_DWELL_MS, 4U);
    else g_deadline = HAL_GetTick() + VISION_DEMO_RESCAN_DWELL_MS;
    EnterState(VISION_ROUTE_DEMO_RESCAN_NUMBER_SIDE);
    return;
  }
  RefreshSurveyMap();
  if (NumberMapValid())
  {
    if (!StartBeanRoute()) EnterFault();
  }
  else if (g_number_retry_used < 2U) BeginNumberRetry();
  else
  {
    g_result_warning = 1U;
    if (!StartBeanRoute()) EnterFault();
  }
}

static void BeginNumberRetry(void)
{
  if (g_number_retry_used >= 2U)
  {
    g_result_warning = 1U;
    if (!StartBeanRoute()) EnterFault();
    return;
  }
  ++g_number_retry_used;
  g_retry_active = 1U;
  g_result_warning = 1U;
  /* 重复语义保留票数更高者；较弱或平票槽位重新定点采集。 */
  InvalidateDuplicateVotes(g_number_votes, WORLD_NUMBER_SLOT_COUNT);
  g_rescan_cursor = 1U;
  ContinueNumberRetry();
}

static void BeginNumberRescanDwell(void)
{
  uint32_t now = HAL_GetTick();
  if (g_debug_step_scan)
  {
    BeginDebugStreamWindow(VISION_DEMO_POINT_SETTLE_MS,
                           VISION_DEMO_RESCAN_DWELL_MS, g_rescan_cursor);
  }
  else
  {
    g_not_before_tick = now;
    g_deadline = now + VISION_DEMO_RESCAN_DWELL_MS;
  }
  EnterState(VISION_ROUTE_DEMO_RESCAN_NUMBER_DWELL);
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
        if (g_debug_step_scan)
          BeginDebugStreamWindow(VISION_DEMO_POINT_SETTLE_MS,
                                 VISION_DEMO_POINT_DWELL_MS, 1U);
        else g_deadline = HAL_GetTick() + VISION_DEMO_POINT_DWELL_MS;
        EnterState(VISION_ROUTE_DEMO_SCAN_NUMBER_START);
      }
      else if (RouteExecutor_GetState() == ROUTE_EXECUTOR_FAULT) EnterFault();
      break;

    case VISION_ROUTE_DEMO_SCAN_NUMBER_START:
      if (!g_debug_step_scan)
      {
        AcceptLatestFrame(K230_TASK_NUMBER);
        if (DeadlineExpired()) EnterState(VISION_ROUTE_DEMO_SCAN_NUMBER_ROW);
      }
      else
      {
        uint8_t stream = ProcessDebugStreamWindow(K230_TASK_NUMBER);
        if (stream == 0U) EnterFault();
        else if (stream == 2U)
        {
          g_debug_row_slot = 2U;
          EnterState(VISION_ROUTE_DEMO_SCAN_NUMBER_ROW);
        }
      }
      break;

    case VISION_ROUTE_DEMO_SCAN_NUMBER_ROW:
      if (g_debug_step_scan)
      {
        int32_t current_x = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
        int32_t target_x = NumberRowSlotX(g_debug_row_slot);
        if (g_stage_started == 0U)
        {
          uint32_t pulses = (uint32_t)((current_x > target_x) ?
                            current_x - target_x : target_x - current_x);
          if ((target_x == WORLD_MAP_UNCALIBRATED) || (pulses == 0U) ||
              (StepperAxis_MovePulses(STEPPER_AXIS_X, pulses,
               (current_x > target_x) ? 1U : 0U) != HAL_OK)) EnterFault();
          else
          {
            g_stage_started = 1U;
            g_deadline = HAL_GetTick() + VISION_DEMO_X_MOVE_TIMEOUT_MS;
          }
        }
        else if (g_stage_started == 1U)
        {
          if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X))
          {
            if (DeadlineExpired()) EnterFault();
            break;
          }
          if ((StepperAxis_GetRemainingPulses(STEPPER_AXIS_X) != 0U) ||
              (StepperAxis_GetPositionPulses(STEPPER_AXIS_X) != target_x))
          {
            EnterFault();
            break;
          }
          WorldMap_SetAxisPosition(target_x, 1U,
              StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
          BeginDebugStreamWindow(VISION_DEMO_POINT_SETTLE_MS,
                                 VISION_DEMO_POINT_DWELL_MS,
                                 g_debug_row_slot);
          g_stage_started = 2U;
        }
        else if (g_stage_started == 2U)
        {
          uint8_t stream = ProcessDebugStreamWindow(K230_TASK_NUMBER);
          if (stream == 0U) EnterFault();
          else if (stream == 2U)
          {
            if (g_debug_row_slot < 3U)
            {
              ++g_debug_row_slot;
              EnterState(VISION_ROUTE_DEMO_SCAN_NUMBER_ROW);
            }
            else
            {
              ServoControl_SetAngle(0U, VISION_DEMO_NUMBER_SIDE_YAW_DEGREES);
              BeginDebugStreamWindow(VISION_DEMO_POINT_SETTLE_MS,
                                     VISION_DEMO_POINT_DWELL_MS, 4U);
              EnterState(VISION_ROUTE_DEMO_SCAN_NUMBER_SIDE);
            }
          }
        }
      }
      else
      {
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
      }
      break;

    case VISION_ROUTE_DEMO_SCAN_NUMBER_SIDE:
      if (g_debug_step_scan)
      {
        uint8_t stream = ProcessDebugStreamWindow(K230_TASK_NUMBER);
        if (stream == 0U) EnterFault();
        else if (stream == 2U)
        {
          RefreshSurveyMap();
          if (NumberMapValid())
          {
            if (!StartBeanRoute()) EnterFault();
          }
          else BeginNumberRetry();
        }
      }
      else
      {
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
          BeginNumberRescanDwell();
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
          BeginNumberRescanDwell();
        }
      }
      else if (DeadlineExpired()) EnterFault();
      break;

    case VISION_ROUTE_DEMO_RESCAN_NUMBER_DWELL:
      if (g_debug_step_scan)
      {
        uint8_t stream = ProcessDebugStreamWindow(K230_TASK_NUMBER);
        if (stream == 0U) EnterFault();
        else if (stream == 2U)
        {
          ++g_rescan_cursor;
          ContinueNumberRetry();
        }
      }
      else
      {
        AcceptLatestFrame(K230_TASK_NUMBER);
        if (DeadlineExpired())
        {
          ++g_rescan_cursor;
          ContinueNumberRetry();
        }
      }
      break;

    case VISION_ROUTE_DEMO_RESCAN_NUMBER_SIDE:
      if (g_debug_step_scan)
      {
        uint8_t stream = ProcessDebugStreamWindow(K230_TASK_NUMBER);
        if (stream == 0U) EnterFault();
        else if (stream == 2U)
        {
          RefreshSurveyMap();
          if (NumberMapValid())
          {
            if (!StartBeanRoute()) EnterFault();
          }
          else if (g_number_retry_used < 2U) BeginNumberRetry();
          else
          {
            g_result_warning = 1U;
            if (!StartBeanRoute()) EnterFault();
          }
        }
      }
      else
      {
        AcceptLatestFrame(K230_TASK_NUMBER);
        if (DeadlineExpired())
        {
          RefreshSurveyMap();
          if (NumberMapValid())
          {
            if (!StartBeanRoute()) EnterFault();
          }
          else if (g_number_retry_used < 2U) BeginNumberRetry();
          else
          {
            g_result_warning = 1U;
            if (!StartBeanRoute()) EnterFault();
          }
        }
      }
      break;

    case VISION_ROUTE_DEMO_MOVE_BEAN:
      if (RouteExecutor_GetState() == ROUTE_EXECUTOR_DONE)
      {
        ServoControl_SetAngle(0U, VISION_DEMO_BEAN_A_YAW_DEGREES);
        CameraTilt_SetDown();
        K230Link_InvalidateResult();
        (void)K230Link_SelectTask(K230_TASK_BEAN);
        if (g_debug_step_scan)
          BeginDebugStreamWindow(VISION_DEMO_POINT_SETTLE_MS,
                                 VISION_DEMO_POINT_DWELL_MS, UINT8_MAX);
        else g_deadline = HAL_GetTick() + VISION_DEMO_POINT_DWELL_MS;
        EnterState(VISION_ROUTE_DEMO_SCAN_BEAN_A);
      }
      else if (RouteExecutor_GetState() == ROUTE_EXECUTOR_FAULT) EnterFault();
      break;

    case VISION_ROUTE_DEMO_SCAN_BEAN_A:
      if (g_debug_step_scan)
      {
        uint8_t stream = ProcessDebugStreamWindow(K230_TASK_BEAN);
        if (stream == 0U) EnterFault();
        else if (stream == 2U)
        {
          ServoControl_SetAngle(0U, VISION_DEMO_BEAN_B_YAW_DEGREES);
          BeginDebugStreamWindow(VISION_DEMO_POINT_SETTLE_MS,
              g_retry_active ? VISION_DEMO_RESCAN_DWELL_MS :
                               VISION_DEMO_POINT_DWELL_MS,
              UINT8_MAX);
          EnterState(VISION_ROUTE_DEMO_SCAN_BEAN_B);
        }
      }
      else
      {
        AcceptLatestFrame(K230_TASK_BEAN);
        if (DeadlineExpired())
        {
          ServoControl_SetAngle(0U, VISION_DEMO_BEAN_B_YAW_DEGREES);
          g_deadline = HAL_GetTick() + (g_retry_active ?
                       VISION_DEMO_RESCAN_DWELL_MS : VISION_DEMO_POINT_DWELL_MS);
          EnterState(VISION_ROUTE_DEMO_SCAN_BEAN_B);
        }
      }
      break;

    case VISION_ROUTE_DEMO_SCAN_BEAN_B:
    {
      uint8_t capture_done = 0U;
      if (g_debug_step_scan)
      {
        uint8_t stream = ProcessDebugStreamWindow(K230_TASK_BEAN);
        if (stream == 0U)
        {
          EnterFault();
          break;
        }
        capture_done = (stream == 2U) ? 1U : 0U;
      }
      else
      {
        AcceptLatestFrame(K230_TASK_BEAN);
        capture_done = DeadlineExpired();
      }
      if (capture_done)
      {
        RefreshSurveyMap();
        if (BeanMapValid())
        {
          FinishBeanScan();
        }
        else if (g_bean_retry_used < 2U)
        {
          ++g_bean_retry_used;
          g_retry_active = 1U;
          g_result_warning = 1U;
          /* 保留已锁存多数结果，补扫继续累计，避免串口短时无数据清空可信槽。 */
          InvalidateDuplicateVotes(g_bean_votes, WORLD_BEAN_SLOT_COUNT);
          ServoControl_SetAngle(0U, VISION_DEMO_BEAN_A_YAW_DEGREES);
          if (g_debug_step_scan)
            BeginDebugStreamWindow(VISION_DEMO_POINT_SETTLE_MS,
                                   VISION_DEMO_RESCAN_DWELL_MS, UINT8_MAX);
          else g_deadline = HAL_GetTick() + VISION_DEMO_RESCAN_DWELL_MS;
          EnterState(VISION_ROUTE_DEMO_SCAN_BEAN_A);
        }
        else
        {
          g_result_warning = 1U;
          FinishBeanScan();
        }
      }
      break;
    }

    case VISION_ROUTE_DEMO_POST_PREPARE:
      if (!g_stage_started)
      {
        uint8_t motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
        if (motion == 0U) EnterFault();
        else if (motion == 2U)
        {
          WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X),
                                   1U, 0, 1U);
          EnterState(VISION_ROUTE_DEMO_POST_MOVE_PICK);
        }
        else
        {
          g_stage_started = 1U;
          g_deadline = HAL_GetTick() + VISION_DEMO_Z_TIMEOUT_MS;
        }
      }
      else if (AxisReached(STEPPER_AXIS_Z, 0))
      {
        WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X),
                                 1U, 0, 1U);
        EnterState(VISION_ROUTE_DEMO_POST_MOVE_PICK);
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z) ||
               DeadlineExpired()) EnterFault();
      break;

    case VISION_ROUTE_DEMO_POST_MOVE_PICK:
      if (!g_stage_started)
      {
        g_stage_started = MissionNavigator_StartWithPayload(
            WORLD_SLOT_BEAN_TOP_LEFT, MISSION_PAYLOAD_EMPTY);
        if (!g_stage_started) EnterFault();
      }
      else if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        EnterState(VISION_ROUTE_DEMO_POST_PICK);
      else if (MissionNavigator_GetState() == MISSION_NAV_FAULT) EnterFault();
      break;

    case VISION_ROUTE_DEMO_POST_PICK:
      if (!g_stage_started)
      {
        g_stage_started = MissionAction_StartPickup(WORLD_SLOT_BEAN_TOP_LEFT);
        if (!g_stage_started) EnterFault();
      }
      else if (MissionAction_GetState() == MISSION_ACTION_DONE)
        EnterState(VISION_ROUTE_DEMO_POST_MOVE_DROP);
      else if (MissionAction_GetState() == MISSION_ACTION_FAULT) EnterFault();
      break;

    case VISION_ROUTE_DEMO_POST_MOVE_DROP:
      if (!g_stage_started)
      {
        /* A到物理4号箱属于跨区导航，MissionNavigator强制经起点换边。 */
        g_stage_started = MissionNavigator_StartWithPayload(
            WORLD_SLOT_NUMBER_BOTTOM_LEFT, MISSION_PAYLOAD_LOADED);
        if (!g_stage_started) EnterFault();
      }
      else if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        EnterState(VISION_ROUTE_DEMO_POST_DROP);
      else if (MissionNavigator_GetState() == MISSION_NAV_FAULT) EnterFault();
      break;

    case VISION_ROUTE_DEMO_POST_DROP:
      if (!g_stage_started)
      {
        g_stage_started = MissionAction_StartDrop(WORLD_SLOT_NUMBER_BOTTOM_LEFT);
        if (!g_stage_started) EnterFault();
      }
      else if (MissionAction_GetState() == MISSION_ACTION_DONE)
        EnterState(VISION_ROUTE_DEMO_POST_RETURN_START);
      else if (MissionAction_GetState() == MISSION_ACTION_FAULT) EnterFault();
      break;

    case VISION_ROUTE_DEMO_POST_RETURN_START:
      if (!g_stage_started)
      {
        g_stage_started = MissionNavigator_StartStationWithPayload(
            WORLD_STATION_START, (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U),
            MISSION_PAYLOAD_EMPTY);
        if (!g_stage_started) EnterFault();
      }
      else if (MissionNavigator_GetState() == MISSION_NAV_DONE)
        EnterState(VISION_ROUTE_DEMO_POST_CENTER_X);
      else if (MissionNavigator_GetState() == MISSION_NAV_FAULT) EnterFault();
      break;

    case VISION_ROUTE_DEMO_POST_CENTER_X:
      if (!g_stage_started)
      {
        uint8_t motion = MoveAxisTo(
            STEPPER_AXIS_X, (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U));
        if (motion == 0U) EnterFault();
        else if (motion == 2U) g_stage_started = 2U;
        else
        {
          g_stage_started = 1U;
          g_deadline = HAL_GetTick() + VISION_DEMO_X_MOVE_TIMEOUT_MS;
        }
      }
      if ((g_stage_started == 2U) ||
          AxisReached(STEPPER_AXIS_X,
                      (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U)))
      {
        AppConfig config;
        AppConfig_GetSnapshot(&config);
        WorldMap_SetAxisPosition((int32_t)(STEPPER_X_TRAVEL_PULSES / 2U),
                                 1U, 0, 1U);
        ServoControl_SetAngle(0U, config.servo_initial_degrees[0]);
        ServoControl_SetAngle(1U, config.gripper_closed_degrees);
        CameraTilt_SetLevel();
        EnterState(VISION_ROUTE_DEMO_POST_HOME_Z);
      }
      else if ((g_stage_started == 1U) &&
               (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X) ||
                DeadlineExpired())) EnterFault();
      break;

    case VISION_ROUTE_DEMO_POST_HOME_Z:
      if (!g_stage_started)
      {
        if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
        {
          EnterFault();
          break;
        }
        if (StepperAxis_MovePulses(STEPPER_AXIS_Z,
                                   STEPPER_Z_TRAVEL_PULSES, 0U) != HAL_OK)
          EnterFault();
        else
        {
          g_stage_started = 1U;
          g_deadline = HAL_GetTick() + VISION_DEMO_Z_TIMEOUT_MS;
        }
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z))
      {
        if ((PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) == 0U) ||
            !StepperAxis_SetPositionPulses(
                STEPPER_AXIS_Z, (int32_t)STEPPER_Z_TRAVEL_PULSES))
          EnterFault();
        else
        {
          WorldMap_SetAxisPosition(
              (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U), 1U,
              (int32_t)STEPPER_Z_TRAVEL_PULSES, 1U);
          WorldMap_SetKnownStation(WORLD_STATION_START);
          EnterState(VISION_ROUTE_DEMO_COMPLETE);
        }
      }
      else if (DeadlineExpired()) EnterFault();
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
         (g_state <= VISION_ROUTE_DEMO_POST_HOME_Z);
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
  /* 路线在底部横排依次采集2/3/4，侧向姿态只采集物理5号箱。
   * 物理1号箱不直接采集，由数字1~5唯一性在RefreshSurveyMap()中补齐。 */
  for (uint8_t i = 0U; i < result->count; ++i)
  {
    const K230VisionTarget *target = &result->targets[i];
    if (!IsExpectedSemantic(K230_TASK_NUMBER, target->semantic)) continue;
    if (NumberSemanticUsedByOtherSlot(target->semantic, 4U)) continue;
    AcceptVote(&g_number_votes[4], target->semantic, 100U);
    break;
  }
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
  uint8_t number_seen = 0U;
  uint8_t bean_seen = 0U;
  uint8_t missing_slot = 2U;
  /* 物理1号从不直接采集，每次都根据2/3/4/5当前锁存结果重算。 */
  memset(&g_number_votes[0], 0, sizeof(g_number_votes[0]));
  memset(&g_survey_map, 0, sizeof(g_survey_map));
  g_survey_map.last_sequence = g_last_sequence;
  for (uint8_t slot = 0U; slot < 5U; ++slot)
  {
    if (!g_number_votes[slot].stable) continue;
    g_survey_map.number_at_slot[slot] = g_number_votes[slot].candidate;
    g_survey_map.number_valid_mask |= (uint8_t)(1U << slot);
    if ((g_number_votes[slot].candidate >= K230_SEMANTIC_NUMBER_1) &&
        (g_number_votes[slot].candidate <= K230_SEMANTIC_NUMBER_5))
      number_seen |= (uint8_t)(1U << (g_number_votes[slot].candidate - 1U));
  }
  /* 物理2/3/4/5已识别且四个数字不重复时，排除得到物理1号箱。 */
  if (g_survey_map.number_valid_mask == 0x1EU)
  {
    uint8_t missing_mask = (uint8_t)(VISION_NUMBER_COMPLETE_MASK & ~number_seen);
    if ((missing_mask != 0U) && ((missing_mask & (uint8_t)(missing_mask - 1U)) == 0U))
    {
      uint8_t missing_number = 1U;
      while ((missing_mask & 0x01U) == 0U)
      {
        missing_mask >>= 1U;
        ++missing_number;
      }
      g_number_votes[0].candidate = missing_number;
      g_number_votes[0].hits = VISION_DEMO_STABLE_HITS;
      g_number_votes[0].confidence = 100U;
      g_number_votes[0].stable = 1U;
      g_survey_map.number_at_slot[0] = missing_number;
      g_survey_map.number_valid_mask |= 0x01U;
      RebuildDisplayResult(K230_TASK_NUMBER, g_number_votes, 5U);
    }
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

/* 正式连续扫描仍用龙门绝对X把字符归到物理2/3/4号箱。
 * 新协议不含坐标和置信度，因此只对当前最近物理槽做纯次数投票。 */
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
    if (!IsExpectedSemantic(K230_TASK_NUMBER, target->semantic)) continue;
    if (NumberSemanticUsedByOtherSlot(target->semantic,
                                      physical_slot[nearest])) continue;
    AcceptVote(&g_number_votes[physical_slot[nearest]], target->semantic, 100U);
    break;
  }
}

uint8_t VisionRouteDemo_GetResultWarning(void)
{
  return g_result_warning;
}

uint8_t VisionRouteDemo_IsComplete(void)
{
  return g_state == VISION_ROUTE_DEMO_COMPLETE;
}

uint8_t VisionRouteDemo_IsRetrying(void)
{
  return g_retry_active;
}

const VisionSurveyMap *VisionRouteDemo_GetSurveyMap(void)
{
  return &g_survey_map;
}

uint8_t VisionRouteDemo_GetNumberTrustedMask(void)
{
  return g_survey_map.number_valid_mask;
}

uint8_t VisionRouteDemo_GetBeanTrustedMask(void)
{
  return g_survey_map.bean_valid_mask;
}
