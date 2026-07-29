#include "xy_waypoint_demo.h"

#include "app_config.h"
#include "bean_pickup_demo.h"
#include "chassis_motion.h"
#include "photo_sensor.h"
#include "safety_manager.h"
#include "stepper_axis.h"

#define XY_WAYPOINT_CHASSIS_RPM             50U
#define XY_WAYPOINT_PER_LANDMARK_TIMEOUT_MS  7000U
#define XY_WAYPOINT_X_SETTLE_MS               300U

typedef struct
{
  const char *name;
  WorldSlotId slot;
} XyWaypointDefinition;

static const XyWaypointDefinition k_waypoints[XY_WAYPOINT_COUNT] = {
  {"A",  WORLD_SLOT_BEAN_TOP_LEFT},
  {"B",  WORLD_SLOT_BEAN_OFFSET},
  {"C",  WORLD_SLOT_BEAN_TOP_RIGHT},
  {"N1", WORLD_SLOT_NUMBER_BOTTOM_LEFT},
  {"N2", WORLD_SLOT_NUMBER_BOTTOM_CENTER},
  {"N3", WORLD_SLOT_NUMBER_BOTTOM_RIGHT},
  {"N4", WORLD_SLOT_NUMBER_OFFSET_LEFT},
  {"N5", WORLD_SLOT_NUMBER_OFFSET_RIGHT},
};

static XyWaypointDemoState g_state;
static XyWaypointId g_selected;
static XyWaypointId g_current_waypoint;
static WorldStationId g_current_station;
static WorldStationId g_target_station;
static int32_t g_target_x;
static uint8_t g_reference_valid;
static uint8_t g_current_waypoint_valid;
static uint8_t g_stage_started;
static uint32_t g_deadline;

static uint8_t HasSafetyFault(void)
{
  return (SafetyManager_GetFlags() &
          (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) ? 1U : 0U;
}

static void EnterState(XyWaypointDemoState state)
{
  g_state = state;
  g_stage_started = 0U;
}

static void EnterFault(void)
{
  ChassisMotion_Stop();
  BeanPickupDemo_Abort();
  StepperAxis_StopAll();
  g_reference_valid = 0U;
  g_current_waypoint_valid = 0U;
  EnterState(XY_WAYPOINT_DEMO_FAULT);
}

static uint8_t LoadSelectedTarget(void)
{
  const WorldSlotPose *slot;
  if (g_selected >= XY_WAYPOINT_COUNT) return 0U;
  slot = WorldMap_GetSlot(k_waypoints[g_selected].slot);
  if ((slot == 0) || !slot->calibrated) return 0U;
  g_target_station = slot->station;
  g_target_x = slot->gantry_x_pulses;
  return 1U;
}

static void StartRouteToSelected(void)
{
  uint8_t landmark_count;
  uint8_t reverse;

  if (!g_reference_valid || !LoadSelectedTarget() ||
      (StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) != 0))
  {
    EnterFault();
    return;
  }

  if (g_target_station == g_current_station)
  {
    StepperAxis_SetDirectionReverse(
        STEPPER_AXIS_X,
        (StepperAxis_GetPositionPulses(STEPPER_AXIS_X) > g_target_x) ? 1U : 0U);
    g_deadline = HAL_GetTick() + XY_WAYPOINT_X_SETTLE_MS;
    EnterState(XY_WAYPOINT_DEMO_X_SETTLE);
    return;
  }

  reverse = (g_target_station < g_current_station) ? 1U : 0U;
  landmark_count = (uint8_t)((g_target_station > g_current_station) ?
      (g_target_station - g_current_station) :
      (g_current_station - g_target_station));
  if (!ChassisMotion_StartPhotoLandmarkRoute(
          reverse, landmark_count, XY_WAYPOINT_CHASSIS_RPM,
          (uint16_t)(landmark_count * XY_WAYPOINT_PER_LANDMARK_TIMEOUT_MS)))
  {
    EnterFault();
    return;
  }
  EnterState(XY_WAYPOINT_DEMO_MOVE_Y);
}

static uint8_t MoveXToTarget(void)
{
  int32_t current = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
  uint32_t pulses;
  uint8_t reverse;
  if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X)) return 1U;
  if (current == g_target_x) return 2U;
  if (g_stage_started &&
      ((g_target_x == 0) ||
       (g_target_x == (int32_t)STEPPER_X_TRAVEL_PULSES)) &&
      (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U) &&
      StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_X))
  {
    /* 数字区侧面4/5号箱就在X两端，硬限位触发点优先于脉冲估算。 */
    if (!StepperAxis_SetPositionPulses(STEPPER_AXIS_X, g_target_x)) return 0U;
    return 2U;
  }
  reverse = (current > g_target_x) ? 1U : 0U;
  pulses = (uint32_t)((current > g_target_x) ?
                      (current - g_target_x) : (g_target_x - current));
  if (StepperAxis_MovePulses(STEPPER_AXIS_X, pulses, reverse) != HAL_OK)
    return 0U;
  g_stage_started = 1U;
  return 1U;
}

void XyWaypointDemo_Init(void)
{
  g_selected = XY_WAYPOINT_A;
  g_current_waypoint = XY_WAYPOINT_A;
  g_current_station = WORLD_STATION_START;
  g_target_station = WORLD_STATION_START;
  g_target_x = 0;
  g_reference_valid = 0U;
  g_current_waypoint_valid = 0U;
  EnterState(XY_WAYPOINT_DEMO_IDLE);
}

void XyWaypointDemo_Select(XyWaypointId target)
{
  if ((target < XY_WAYPOINT_COUNT) && !XyWaypointDemo_IsRunning())
    g_selected = target;
}

void XyWaypointDemo_HandlePower(void)
{
  if (XyWaypointDemo_IsRunning() || (g_state == XY_WAYPOINT_DEMO_FAULT)) return;
  if (!g_reference_valid)
  {
    if (HasSafetyFault() || ChassisMotion_IsRunning() ||
        StepperAxis_IsEnabled(STEPPER_AXIS_X) ||
        StepperAxis_IsEnabled(STEPPER_AXIS_Z) ||
        (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) == 0U) ||
        (PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_ORIGIN_SIDE) == 0U) ||
        (PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_FAR_SIDE) == 0U))
    {
      EnterFault();
      return;
    }
    ChassisMotion_Stop();
    BeanPickupDemo_Abort();
    (void)StepperAxis_SetPositionPulses(
        STEPPER_AXIS_X, (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U));
    (void)StepperAxis_SetPositionPulses(
        STEPPER_AXIS_Z, (int32_t)STEPPER_Z_TRAVEL_PULSES);
    WorldMap_SetPoseAtStart();
    BeanPickupDemo_HandlePower();
    EnterState(XY_WAYPOINT_DEMO_REFERENCE);
    return;
  }
  StartRouteToSelected();
}

void XyWaypointDemo_Abort(void)
{
  ChassisMotion_Stop();
  BeanPickupDemo_Abort();
  StepperAxis_StopAll();
  /* 中途停止后Y地标不再可信，必须回到起点规定姿态重新建立参考。 */
  g_reference_valid = 0U;
  g_current_waypoint_valid = 0U;
  EnterState(XY_WAYPOINT_DEMO_IDLE);
}

void XyWaypointDemo_Process(void)
{
  uint8_t motion;
  if (!XyWaypointDemo_IsRunning()) return;
  StepperAxis_ProcessPhotoInterlock();
  if (HasSafetyFault())
  {
    EnterFault();
    return;
  }

  switch (g_state)
  {
    case XY_WAYPOINT_DEMO_REFERENCE:
      if (BeanPickupDemo_GetState() == BEAN_PICKUP_DEMO_FAULT) EnterFault();
      else if (BeanPickupDemo_IsStartReady())
      {
        g_current_station = WORLD_STATION_START;
        g_reference_valid = 1U;
        g_current_waypoint_valid = 0U;
        StartRouteToSelected();
      }
      break;

    case XY_WAYPOINT_DEMO_MOVE_Y:
      if (ChassisMotion_DidRouteSegmentFail()) EnterFault();
      else if (ChassisMotion_IsRouteSegmentDone())
      {
        g_current_station = g_target_station;
        StepperAxis_SetDirectionReverse(
            STEPPER_AXIS_X,
            (StepperAxis_GetPositionPulses(STEPPER_AXIS_X) > g_target_x) ? 1U : 0U);
        g_deadline = HAL_GetTick() + XY_WAYPOINT_X_SETTLE_MS;
        EnterState(XY_WAYPOINT_DEMO_X_SETTLE);
      }
      break;

    case XY_WAYPOINT_DEMO_X_SETTLE:
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(XY_WAYPOINT_DEMO_MOVE_X);
      break;

    case XY_WAYPOINT_DEMO_MOVE_X:
      motion = MoveXToTarget();
      if (motion == 2U)
      {
        WorldMap_SetAxisPosition(
            StepperAxis_GetPositionPulses(STEPPER_AXIS_X), 1U, 0, 1U);
        g_current_waypoint = g_selected;
        g_current_waypoint_valid = 1U;
        EnterState(XY_WAYPOINT_DEMO_COMPLETE);
      }
      else if (motion == 0U) EnterFault();
      break;

    default:
      break;
  }
}

XyWaypointDemoState XyWaypointDemo_GetState(void)
{
  return g_state;
}

XyWaypointId XyWaypointDemo_GetSelected(void)
{
  return g_selected;
}

WorldStationId XyWaypointDemo_GetCurrentStation(void)
{
  return g_current_station;
}

const char *XyWaypointDemo_GetCurrentName(void)
{
  if (!g_reference_valid) return "未回零";
  if (!g_current_waypoint_valid) return "起点";
  return XyWaypointDemo_GetName(g_current_waypoint);
}

uint8_t XyWaypointDemo_HasReference(void)
{
  return g_reference_valid;
}

uint8_t XyWaypointDemo_IsRunning(void)
{
  return (g_state == XY_WAYPOINT_DEMO_REFERENCE) ||
         (g_state == XY_WAYPOINT_DEMO_MOVE_Y) ||
         (g_state == XY_WAYPOINT_DEMO_X_SETTLE) ||
         (g_state == XY_WAYPOINT_DEMO_MOVE_X);
}

const char *XyWaypointDemo_GetName(XyWaypointId target)
{
  return (target < XY_WAYPOINT_COUNT) ? k_waypoints[target].name : "?";
}
