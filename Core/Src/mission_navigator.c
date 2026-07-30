#include "mission_navigator.h"

#include "app_config.h"
#include "chassis_motion.h"
#include "safety_manager.h"
#include "stepper_axis.h"

#define MISSION_NAV_PER_LANDMARK_TIMEOUT_MS 7000U

static MissionNavigatorState g_state;
static WorldStationId g_target_station;
static WorldStationId g_leg_target_station;
static int32_t g_target_x;
static int32_t g_cross_target_x;
static uint8_t g_cross_zone;

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
  if (g_state == MISSION_NAV_CROSS_X) WorldMap_InvalidateX();
  ChassisMotion_Stop();
  StepperAxis_StopAll();
  WorldMap_InvalidateY();
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
  int32_t midpoint = (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U);
  uint32_t pulses;

  g_cross_target_x = (g_target_x < midpoint) ?
      (int32_t)(STEPPER_X_TRAVEL_PULSES / 4U) :
      (int32_t)(STEPPER_X_TRAVEL_PULSES * 3U / 4U);
  pulses = (uint32_t)((current > g_cross_target_x) ?
           (current - g_cross_target_x) : (g_cross_target_x - current));
  if (pulses == 0U)
  {
    g_cross_zone = 0U;
    return StartLeg(g_target_station);
  }
  if (StepperAxis_MovePulses(STEPPER_AXIS_X, pulses,
      (current > g_cross_target_x) ? 1U : 0U) != HAL_OK) return 0U;
  g_state = MISSION_NAV_CROSS_X;
  return 1U;
}

void MissionNavigator_Init(void)
{
  g_target_station = WORLD_STATION_START;
  g_leg_target_station = WORLD_STATION_START;
  g_target_x = 0;
  g_cross_target_x = 0;
  g_cross_zone = 0U;
  g_state = MISSION_NAV_IDLE;
}

uint8_t MissionNavigator_Start(WorldSlotId target_slot)
{
  const WorldSlotPose *slot = WorldMap_GetSlot(target_slot);
  if ((slot == 0) || !slot->calibrated) return 0U;
  return MissionNavigator_StartStation(slot->station, slot->gantry_x_pulses);
}

uint8_t MissionNavigator_StartStation(WorldStationId target_station,
                                      int32_t target_x_pulses)
{
  const WorldPose *pose = WorldMap_GetPose();
  if ((g_state == MISSION_NAV_MOVE_Y) || (g_state == MISSION_NAV_CROSS_X) ||
      !WorldMap_IsTopologyTrusted() || !WorldMap_IsPoseValid() ||
      (StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) != 0) ||
      (target_station >= WORLD_STATION_COUNT) ||
      (target_x_pulses < 0) ||
      (target_x_pulses > (int32_t)STEPPER_X_TRAVEL_PULSES)) return 0U;

  g_target_station = target_station;
  g_target_x = target_x_pulses;
  g_cross_zone =
      ((IsBeanZone(pose->station) && IsNumberZone(target_station)) ||
       (IsNumberZone(pose->station) && IsBeanZone(target_station))) ? 1U : 0U;
  if (pose->station == target_station)
  {
    g_state = MISSION_NAV_DONE;
    return 1U;
  }
  if (g_cross_zone)
  {
    if (pose->station == WORLD_STATION_START)
      return StartCross();
    return StartLeg(WORLD_STATION_START);
  }
  return StartLeg(target_station);
}

void MissionNavigator_Process(void)
{
  if ((g_state != MISSION_NAV_MOVE_Y) &&
      (g_state != MISSION_NAV_CROSS_X)) return;
  if ((SafetyManager_GetFlags() & (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) != 0U)
  {
    EnterFault();
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
  if (!StartLeg(g_target_station)) EnterFault();
}

void MissionNavigator_Abort(void)
{
  ChassisMotion_Stop();
  if (g_state == MISSION_NAV_CROSS_X) WorldMap_InvalidateX();
  if (g_state == MISSION_NAV_MOVE_Y) WorldMap_InvalidateY();
  StepperAxis_StopAll();
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
