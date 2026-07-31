#include "mission_navigator.h"

#include "app_config.h"
#include "chassis_motion.h"
#include "photo_sensor.h"
#include "safety_manager.h"
#include "stepper_axis.h"

#include <string.h>

#define MISSION_NAV_PER_LANDMARK_TIMEOUT_MS 8500U
#define MISSION_NAV_SAFE_CLEAR_STABLE_MS     100U
#define MISSION_NAV_CENTER_BAND_PULSES       2000

static MissionNavigatorState g_state;
static WorldStationId g_target_station;
static WorldStationId g_leg_target_station;
static int32_t g_target_x;
static int32_t g_cross_target_x;
static uint8_t g_cross_zone;
static uint8_t g_parallel_y_done;
static uint8_t g_parallel_x_started;
static uint8_t g_parallel_x_trigger_count;
static uint8_t g_safe_clear_timing;
static uint32_t g_safe_clear_since;
static MissionNavigatorCrossPhase g_cross_phase;
static MissionRoutePlan g_plan;
static int32_t g_entry_lane_x;
static int32_t g_exit_lane_x;

static uint8_t IsBeanZone(WorldStationId station);
static uint8_t IsNumberZone(WorldStationId station);

static uint32_t AbsPulseDifference(int32_t a, int32_t b)
{
  return (uint32_t)((a > b) ? (a - b) : (b - a));
}

static int8_t StationZone(WorldStationId station)
{
  if (IsBeanZone(station)) return -1;
  if (IsNumberZone(station)) return 1;
  return 0;
}

static int32_t LaneX(MissionRouteLane lane)
{
  return (lane == MISSION_LANE_LEFT) ?
      (int32_t)(STEPPER_X_TRAVEL_PULSES * 3U / 4U) :
      (int32_t)(STEPPER_X_TRAVEL_PULSES / 4U);
}

static MissionRouteLane ClassifyLane(int32_t x)
{
  int32_t midpoint = (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U);
  if (x < midpoint - MISSION_NAV_CENTER_BAND_PULSES) return MISSION_LANE_RIGHT;
  if (x > midpoint + MISSION_NAV_CENTER_BAND_PULSES) return MISSION_LANE_LEFT;
  return MISSION_LANE_NONE;
}

static uint8_t BuildRoutePlan(const WorldPose *pose,
                              WorldStationId target_station,
                              int32_t target_x, MissionPayloadState payload)
{
  int8_t source_zone;
  int8_t target_zone;
  MissionRouteLane source_side;
  MissionRouteLane target_side;
  uint32_t best_cost = UINT32_MAX;
  uint8_t best_switches = UINT8_MAX;

  memset(&g_plan, 0, sizeof(g_plan));
  g_plan.payload = payload;
  if ((pose == 0) || !pose->x_valid || !pose->y_valid || !pose->z_valid)
  {
    g_plan.fault = MISSION_ROUTE_PLAN_INVALID_POSE;
    return 0U;
  }
  if (payload == MISSION_PAYLOAD_UNKNOWN)
  {
    g_plan.fault = MISSION_ROUTE_PLAN_UNKNOWN_PAYLOAD;
    return 0U;
  }
  source_zone = StationZone(pose->station);
  target_zone = StationZone(target_station);
  if ((source_zone == target_zone) && (source_zone != 0))
  {
    g_plan.type = MISSION_ROUTE_SAME_ZONE;
    g_plan.fault = MISSION_ROUTE_PLAN_OK;
    return 1U;
  }
  if ((pose->station == target_station) ||
      ((source_zone == 0) && (target_zone == 0)))
  {
    g_plan.type = MISSION_ROUTE_SAME_ZONE;
    g_plan.fault = MISSION_ROUTE_PLAN_OK;
    return 1U;
  }

  source_side = ClassifyLane(pose->x_pulses);
  target_side = ClassifyLane(target_x);
  for (MissionRouteLane entry = MISSION_LANE_LEFT;
       entry <= MISSION_LANE_RIGHT; ++entry)
  {
    for (MissionRouteLane exit_lane = MISSION_LANE_LEFT;
         exit_lane <= MISSION_LANE_RIGHT; ++exit_lane)
    {
      uint8_t full_cross = (source_zone != 0) && (target_zone != 0) &&
                           (source_zone != target_zone);
      uint8_t switches = (entry != exit_lane) ? 1U : 0U;
      uint32_t cost;

      if (!full_cross && (entry != exit_lane)) continue;
      if (full_cross && (payload == MISSION_PAYLOAD_LOADED) &&
          (entry == exit_lane)) continue;
      if (full_cross && (payload == MISSION_PAYLOAD_EMPTY))
      {
        if ((source_side != MISSION_LANE_NONE) && (entry != source_side)) continue;
        if ((target_side != MISSION_LANE_NONE) && (exit_lane != target_side)) continue;
        if ((source_side != MISSION_LANE_NONE) &&
            (target_side != MISSION_LANE_NONE) &&
            (source_side == target_side) && (entry != exit_lane)) continue;
        if ((source_side != MISSION_LANE_NONE) &&
            (target_side != MISSION_LANE_NONE) &&
            (source_side != target_side) && (entry == exit_lane)) continue;
      }
      cost = AbsPulseDifference(pose->x_pulses, LaneX(entry)) +
             AbsPulseDifference(LaneX(entry), LaneX(exit_lane)) +
             AbsPulseDifference(LaneX(exit_lane), target_x);
      if ((cost < best_cost) ||
          ((cost == best_cost) && (switches < best_switches)))
      {
        best_cost = cost;
        best_switches = switches;
        g_plan.entry_lane = entry;
        g_plan.exit_lane = exit_lane;
      }
    }
  }
  if (best_cost == UINT32_MAX)
  {
    g_plan.fault = MISSION_ROUTE_PLAN_NO_SAFE_PATH;
    return 0U;
  }
  g_entry_lane_x = LaneX(g_plan.entry_lane);
  g_exit_lane_x = LaneX(g_plan.exit_lane);
  g_plan.x_cost_pulses = best_cost;
  g_plan.requires_center_cross =
      (g_plan.entry_lane != g_plan.exit_lane) ? 1U : 0U;
  g_plan.requires_prealign =
      (pose->x_pulses != g_entry_lane_x) ? 1U : 0U;
  g_plan.type = ((source_zone != 0) && (target_zone != 0) &&
                 (source_zone != target_zone) &&
                 (payload == MISSION_PAYLOAD_LOADED)) ?
      MISSION_ROUTE_LOADED_CENTER :
      g_plan.requires_center_cross ? MISSION_ROUTE_EMPTY_CENTER_CROSS :
      (g_plan.entry_lane == MISSION_LANE_LEFT) ?
          MISSION_ROUTE_EMPTY_LEFT_DIRECT : MISSION_ROUTE_EMPTY_RIGHT_DIRECT;
  g_plan.fault = MISSION_ROUTE_PLAN_OK;
  return 1U;
}

static uint8_t IsBeanZone(WorldStationId station)
{
  return station < WORLD_STATION_START;
}

static uint8_t IsNumberZone(WorldStationId station)
{
  return station > WORLD_STATION_START;
}

static void EnterFault(void)
{
  if ((g_state == MISSION_NAV_CROSS_X) ||
      (g_state == MISSION_NAV_PREALIGN_X) ||
      (g_state == MISSION_NAV_MOVE_Y_CROSS_X)) WorldMap_InvalidateX();
  ChassisMotion_Stop();
  StepperAxis_StopMotionPreserveZ();
  WorldMap_InvalidateY();
  g_cross_phase = MISSION_NAV_CROSS_PHASE_NONE;
  g_state = MISSION_NAV_FAULT;
}

static uint8_t StartLeg(WorldStationId destination)
{
  const WorldPose *pose = WorldMap_GetPose();
  uint8_t count;
  uint8_t reverse;
  uint32_t timeout;
  int16_t rpm = ChassisMotion_GetTargetRpm();

  if (!pose->y_valid || (destination >= WORLD_STATION_COUNT) ||
      (destination == pose->station) || (rpm <= 0)) return 0U;
  reverse = (destination < pose->station) ? 1U : 0U;
  count = (uint8_t)((destination > pose->station) ?
          (destination - pose->station) : (pose->station - destination));
  timeout = (uint32_t)count * MISSION_NAV_PER_LANDMARK_TIMEOUT_MS;
  if (timeout > UINT16_MAX) timeout = UINT16_MAX;
  if (WorldMap_RequiresBlockedAlignment(destination))
  {
    if (!ChassisMotion_StartAlignedPhotoLandmarkRoute(
            reverse, count, (uint16_t)rpm, (uint16_t)timeout)) return 0U;
  }
  else if (!ChassisMotion_StartPhotoLandmarkRoute(
               reverse, count, (uint16_t)rpm, (uint16_t)timeout)) return 0U;
  g_leg_target_station = destination;
  g_state = MISSION_NAV_MOVE_Y;
  return 1U;
}

static uint8_t StartCross(void)
{
  int32_t current = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
  uint32_t pulses;

  /* 跨区时必须在起点把横梁切换到当前所在侧的对侧，不能按最终箱位选侧。
   * 否则A箱和物理4号箱这类X同侧的组合只会产生很小的横移，实车会表现为
   * 没有经过障碍中间的安全换边动作便直接驶向另一分区。 */
  g_cross_target_x = g_exit_lane_x;
  pulses = (uint32_t)((current > g_cross_target_x) ?
           (current - g_cross_target_x) : (g_cross_target_x - current));
  if (pulses == 0U)
  {
    g_cross_zone = 0U;
    return StartLeg(g_target_station);
  }
  if (StepperAxis_MovePulses(STEPPER_AXIS_X, pulses,
      (current > g_cross_target_x) ? 1U : 0U) != HAL_OK) return 0U;
  g_cross_phase = MISSION_NAV_CROSS_PHASE_MOVING_X;
  g_state = MISSION_NAV_CROSS_X;
  return 1U;
}

static WorldStationId GetCrossStagingStation(const WorldPose *pose)
{
  return IsBeanZone(pose->station) ? WORLD_STATION_UPPER_OBSTACLE_LOWER :
                                    WORLD_STATION_LOWER_OBSTACLE_UPPER;
}

/* 仅在障碍物与起点之间的已确认安全直道内并行执行Y前进和X换边。 */
static uint8_t StartParallelCrossToStart(void)
{
  const WorldPose *pose = WorldMap_GetPose();
  WorldStationId staging = GetCrossStagingStation(pose);

  g_cross_target_x = g_exit_lane_x;
  g_parallel_x_trigger_count = (uint8_t)((pose->station > staging) ?
      (pose->station - staging) : (staging - pose->station));
  g_parallel_x_started = 0U;
  g_safe_clear_timing = 0U;
  g_cross_phase = MISSION_NAV_CROSS_PHASE_WAIT_ENTRY;
  /* 一次启动到起点，中途不停车；双侧都通过安全直道入口后才启动X。 */
  if (!StartLeg(WORLD_STATION_START)) return 0U;
  g_parallel_y_done = 0U;
  g_state = MISSION_NAV_MOVE_Y_CROSS_X;
  return 1U;
}

static uint8_t StartParallelX(void)
{
  const WorldPose *pose = WorldMap_GetPose();
  int32_t current = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
  uint32_t pulses = (uint32_t)((current > g_cross_target_x) ?
      (current - g_cross_target_x) : (g_cross_target_x - current));
  if (!pose->x_valid || !pose->z_valid ||
      (StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) != 0) ||
      (g_cross_target_x < 0) ||
      (g_cross_target_x > (int32_t)STEPPER_X_TRAVEL_PULSES)) return 0U;
  if (pulses == 0U)
  {
    g_parallel_x_started = 1U;
    g_cross_phase = MISSION_NAV_CROSS_PHASE_MOVING_X;
    return 1U;
  }
  if (StepperAxis_MovePulses(STEPPER_AXIS_X, pulses,
      (current > g_cross_target_x) ? 1U : 0U) != HAL_OK) return 0U;
  g_parallel_x_started = 1U;
  g_cross_phase = MISSION_NAV_CROSS_PHASE_MOVING_X;
  return 1U;
}

static uint8_t BeginPlannedRoute(void)
{
  const WorldPose *pose = WorldMap_GetPose();
  int8_t source_zone = StationZone(pose->station);
  int8_t target_zone = StationZone(g_target_station);

  g_cross_zone = (source_zone != 0) && (target_zone != 0) &&
                 (source_zone != target_zone) &&
                 g_plan.requires_center_cross;
  if (pose->station == g_target_station)
  {
    g_state = MISSION_NAV_DONE;
    return 1U;
  }
  if (g_cross_zone) return StartParallelCrossToStart();
  return StartLeg(g_target_station);
}

void MissionNavigator_Init(void)
{
  g_target_station = WORLD_STATION_START;
  g_leg_target_station = WORLD_STATION_START;
  g_target_x = 0;
  g_cross_target_x = 0;
  g_cross_zone = 0U;
  g_parallel_y_done = 0U;
  g_parallel_x_started = 0U;
  g_parallel_x_trigger_count = 0U;
  g_safe_clear_timing = 0U;
  g_safe_clear_since = 0U;
  g_cross_phase = MISSION_NAV_CROSS_PHASE_NONE;
  memset(&g_plan, 0, sizeof(g_plan));
  g_plan.payload = MISSION_PAYLOAD_EMPTY;
  g_plan.type = MISSION_ROUTE_SAME_ZONE;
  g_plan.fault = MISSION_ROUTE_PLAN_OK;
  g_entry_lane_x = 0;
  g_exit_lane_x = 0;
  g_state = MISSION_NAV_IDLE;
}

uint8_t MissionNavigator_Start(WorldSlotId target_slot)
{
  return MissionNavigator_StartWithPayload(target_slot, MISSION_PAYLOAD_EMPTY);
}

uint8_t MissionNavigator_StartWithPayload(WorldSlotId target_slot,
                                          MissionPayloadState payload)
{
  const WorldSlotPose *slot = WorldMap_GetSlot(target_slot);
  if ((slot == 0) || !slot->calibrated) return 0U;
  return MissionNavigator_StartStationWithPayload(
      slot->station, slot->gantry_x_pulses, payload);
}

uint8_t MissionNavigator_StartStation(WorldStationId target_station,
                                      int32_t target_x_pulses)
{
  return MissionNavigator_StartStationWithPayload(
      target_station, target_x_pulses, MISSION_PAYLOAD_EMPTY);
}

uint8_t MissionNavigator_StartStationWithPayload(
    WorldStationId target_station, int32_t target_x_pulses,
    MissionPayloadState payload)
{
  const WorldPose *pose = WorldMap_GetPose();
  uint32_t pulses;
  if ((g_state == MISSION_NAV_PREALIGN_X) ||
      (g_state == MISSION_NAV_MOVE_Y) ||
      (g_state == MISSION_NAV_MOVE_Y_CROSS_X) ||
      (g_state == MISSION_NAV_CROSS_X) ||
      !WorldMap_IsTopologyTrusted() || !WorldMap_IsPoseValid() ||
      (StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) != 0) ||
      (StepperAxis_GetRemainingPulses(STEPPER_AXIS_Z) != 0U) ||
      (target_station >= WORLD_STATION_COUNT) ||
      (target_x_pulses < 0) ||
      (target_x_pulses > (int32_t)STEPPER_X_TRAVEL_PULSES)) return 0U;

  g_target_station = target_station;
  g_target_x = target_x_pulses;
  if (!BuildRoutePlan(pose, target_station, target_x_pulses, payload)) return 0U;
  if (pose->station == target_station)
  {
    g_state = MISSION_NAV_DONE;
    return 1U;
  }
  if (g_plan.requires_prealign)
  {
    pulses = AbsPulseDifference(
        StepperAxis_GetPositionPulses(STEPPER_AXIS_X), g_entry_lane_x);
    if ((pulses == 0U) ||
        (StepperAxis_MovePulses(STEPPER_AXIS_X, pulses,
         StepperAxis_GetPositionPulses(STEPPER_AXIS_X) > g_entry_lane_x) != HAL_OK))
      return 0U;
    g_state = MISSION_NAV_PREALIGN_X;
    return 1U;
  }
  return BeginPlannedRoute();
}

void MissionNavigator_Process(void)
{
  if ((g_state != MISSION_NAV_PREALIGN_X) &&
      (g_state != MISSION_NAV_MOVE_Y) &&
      (g_state != MISSION_NAV_MOVE_Y_CROSS_X) &&
      (g_state != MISSION_NAV_CROSS_X)) return;
  if ((SafetyManager_GetFlags() & (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) != 0U)
  {
    EnterFault();
    return;
  }

  if (g_state == MISSION_NAV_PREALIGN_X)
  {
    if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X)) return;
    if ((StepperAxis_GetRemainingPulses(STEPPER_AXIS_X) != 0U) ||
        (StepperAxis_GetPositionPulses(STEPPER_AXIS_X) != g_entry_lane_x))
    {
      EnterFault();
      return;
    }
    WorldMap_SetAxisPosition(g_entry_lane_x, 1U,
                             StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
    if (!BeginPlannedRoute()) EnterFault();
    return;
  }

  if (g_state == MISSION_NAV_MOVE_Y)
  {
    if (ChassisMotion_DidRouteSegmentFail())
    {
      EnterFault();
      return;
    }
    if (!ChassisMotion_IsRouteSegmentDone()) return;
    WorldMap_SetKnownStation(g_leg_target_station);
    if (g_cross_zone && (g_leg_target_station == WORLD_STATION_START))
    {
      if (!StartCross()) EnterFault();
    }
    else g_state = MISSION_NAV_DONE;
    return;
  }

  if (g_state == MISSION_NAV_MOVE_Y_CROSS_X)
  {
    uint8_t origin_count =
        ChassisMotion_GetPassedLandmarkCount(CHASSIS_SIDE_ORIGIN);
    uint8_t far_count = ChassisMotion_GetPassedLandmarkCount(CHASSIS_SIDE_FAR);
    uint8_t entry_confirmed =
        (origin_count >= g_parallel_x_trigger_count) &&
        (far_count >= g_parallel_x_trigger_count);

    if (((origin_count > far_count) ? (origin_count - far_count) :
         (far_count - origin_count)) > 1U)
    {
      EnterFault();
      return;
    }
    if (!g_parallel_x_started && entry_confirmed)
    {
      uint8_t clear =
          (PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_ORIGIN_SIDE) == 0U) &&
          (PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_FAR_SIDE) == 0U);
      g_cross_phase = MISSION_NAV_CROSS_PHASE_WAIT_CLEAR;
      if (!clear) g_safe_clear_timing = 0U;
      else if (!g_safe_clear_timing)
      {
        g_safe_clear_since = HAL_GetTick();
        g_safe_clear_timing = 1U;
      }
      else if ((uint32_t)(HAL_GetTick() - g_safe_clear_since) >=
               MISSION_NAV_SAFE_CLEAR_STABLE_MS)
      {
        if (!StartParallelX())
        {
          EnterFault();
          return;
        }
      }
    }
    if (!g_parallel_y_done)
    {
      if (ChassisMotion_DidRouteSegmentFail())
      {
        EnterFault();
        return;
      }
      if (ChassisMotion_IsRouteSegmentDone())
      {
        WorldMap_SetKnownStation(WORLD_STATION_START);
        g_parallel_y_done = 1U;
        if (!g_parallel_x_started)
        {
          /* 已到起点挡板仍未在安全直道启动X，不能在挡板内补做横移。 */
          EnterFault();
          return;
        }
      }
    }
    if (g_parallel_y_done && StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X))
      g_cross_phase = MISSION_NAV_CROSS_PHASE_WAIT_X_AT_START;
    if (!g_parallel_y_done || !g_parallel_x_started ||
        StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X)) return;
    if ((StepperAxis_GetRemainingPulses(STEPPER_AXIS_X) != 0U) ||
        (StepperAxis_GetPositionPulses(STEPPER_AXIS_X) != g_cross_target_x))
    {
      EnterFault();
      return;
    }
    WorldMap_SetAxisPosition(g_cross_target_x, 1U,
                             StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
    g_cross_zone = 0U;
    g_cross_phase = MISSION_NAV_CROSS_PHASE_NONE;
    if (!StartLeg(g_target_station)) EnterFault();
    return;
  }

  if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X)) return;
  if ((StepperAxis_GetRemainingPulses(STEPPER_AXIS_X) != 0U) ||
      (StepperAxis_GetPositionPulses(STEPPER_AXIS_X) != g_cross_target_x))
  {
    WorldMap_InvalidateX();
    EnterFault();
    return;
  }
  WorldMap_SetAxisPosition(g_cross_target_x, 1U,
                           StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
  g_cross_zone = 0U;
  g_cross_phase = MISSION_NAV_CROSS_PHASE_NONE;
  if (!StartLeg(g_target_station)) EnterFault();
}

void MissionNavigator_Abort(void)
{
  ChassisMotion_Stop();
  if ((g_state == MISSION_NAV_CROSS_X) ||
      (g_state == MISSION_NAV_MOVE_Y_CROSS_X) ||
      (g_state == MISSION_NAV_PREALIGN_X)) WorldMap_InvalidateX();
  if ((g_state == MISSION_NAV_MOVE_Y) ||
      (g_state == MISSION_NAV_MOVE_Y_CROSS_X)) WorldMap_InvalidateY();
  StepperAxis_StopMotionPreserveZ();
  g_cross_phase = MISSION_NAV_CROSS_PHASE_NONE;
  g_state = MISSION_NAV_IDLE;
}

MissionNavigatorState MissionNavigator_GetState(void)
{
  return g_state;
}

WorldStationId MissionNavigator_GetTargetStation(void)
{
  return g_target_station;
}

MissionNavigatorRecoveryState MissionNavigator_GetRecoveryState(void)
{
  ChassisAlignmentState alignment = ChassisMotion_GetAlignmentState();
  if ((alignment == CHASSIS_ALIGNMENT_RECOVERY_SETTLING) ||
      (alignment == CHASSIS_ALIGNMENT_RECOVERING))
    return MISSION_NAV_RECOVERY_RUNNING;
  return (g_state == MISSION_NAV_FAULT) ?
         MISSION_NAV_RECOVERY_FAILED : MISSION_NAV_RECOVERY_NONE;
}

MissionNavigatorCrossPhase MissionNavigator_GetCrossPhase(void)
{
  return g_cross_phase;
}

const MissionRoutePlan *MissionNavigator_GetRoutePlan(void)
{
  return &g_plan;
}
