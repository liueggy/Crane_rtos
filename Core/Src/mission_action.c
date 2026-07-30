#include "mission_action.h"

#include "app_config.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"

#define MISSION_ACTION_SERVO_SETTLE_MS 600U
#define MISSION_ACTION_GRIP_HOLD_MS    900U

typedef enum
{
  ACTION_KIND_PICKUP = 0,
  ACTION_KIND_DROP
} MissionActionKind;

static MissionActionState g_state;
static MissionActionKind g_kind;
static WorldSlotId g_slot_id;
static uint8_t g_stage_started;
static uint32_t g_deadline;

static void EnterState(MissionActionState state)
{
  g_state = state;
  g_stage_started = 0U;
}

static void EnterFault(void)
{
  if ((g_state == MISSION_ACTION_MOVE_X) ||
      StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X)) WorldMap_InvalidateX();
  if ((g_state == MISSION_ACTION_LIFT) ||
      (g_state == MISSION_ACTION_DESCEND) ||
      (g_state == MISSION_ACTION_RAISE) ||
      StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z)) WorldMap_InvalidateZ();
  StepperAxis_StopAll();
  EnterState(MISSION_ACTION_FAULT);
}

/* 0失败，1运动中，2已到位。 */
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

static uint8_t Start(WorldSlotId slot_id, MissionActionKind kind)
{
  const WorldPose *pose = WorldMap_GetPose();
  const WorldSlotPose *slot = WorldMap_GetSlot(slot_id);
  if ((g_state >= MISSION_ACTION_LIFT) &&
      (g_state <= MISSION_ACTION_RESET_TOOL)) return 0U;
  if ((slot == 0) || !slot->calibrated || !pose->x_valid || !pose->z_valid ||
      !pose->y_valid || (pose->station != slot->station) ||
      StepperAxis_IsEnabled(STEPPER_AXIS_X) ||
      StepperAxis_IsEnabled(STEPPER_AXIS_Z)) return 0U;
  g_kind = kind;
  g_slot_id = slot_id;
  EnterState(MISSION_ACTION_LIFT);
  return 1U;
}

void MissionAction_Init(void)
{
  g_kind = ACTION_KIND_PICKUP;
  g_slot_id = WORLD_SLOT_BEAN_TOP_LEFT;
  EnterState(MISSION_ACTION_IDLE);
}

uint8_t MissionAction_StartPickup(WorldSlotId slot)
{
  if (slot > WORLD_SLOT_BEAN_OFFSET) return 0U;
  return Start(slot, ACTION_KIND_PICKUP);
}

uint8_t MissionAction_StartDrop(WorldSlotId slot)
{
  if (slot < WORLD_SLOT_NUMBER_BOTTOM_LEFT) return 0U;
  return Start(slot, ACTION_KIND_DROP);
}

void MissionAction_Process(void)
{
  AppConfig config;
  const WorldSlotPose *slot;
  uint8_t motion;
  if ((g_state < MISSION_ACTION_LIFT) ||
      (g_state > MISSION_ACTION_RESET_TOOL)) return;
  if ((SafetyManager_GetFlags() & (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) != 0U)
  {
    EnterFault();
    return;
  }
  StepperAxis_ProcessPhotoInterlock();
  slot = WorldMap_GetSlot(g_slot_id);
  if ((slot == 0) || !slot->calibrated)
  {
    EnterFault();
    return;
  }
  AppConfig_GetSnapshot(&config);

  switch (g_state)
  {
    case MISSION_ACTION_LIFT:
      motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
      if (motion == 2U) EnterState(MISSION_ACTION_PREPARE_TOOL);
      else if (motion == 0U) EnterFault();
      break;

    case MISSION_ACTION_PREPARE_TOOL:
      if (!g_stage_started)
      {
        ServoControl_SetAngle(0U, slot->tool_yaw_degrees);
        if (g_kind == ACTION_KIND_PICKUP)
          ServoControl_SetAngle(1U, config.gripper_open_degrees);
        g_deadline = HAL_GetTick() + MISSION_ACTION_SERVO_SETTLE_MS;
        g_stage_started = 1U;
      }
      else if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(MISSION_ACTION_MOVE_X);
      break;

    case MISSION_ACTION_MOVE_X:
      motion = MoveAxisTo(STEPPER_AXIS_X, slot->gantry_x_pulses);
      if (motion == 2U) EnterState(MISSION_ACTION_DESCEND);
      else if (motion == 0U) EnterFault();
      break;

    case MISSION_ACTION_DESCEND:
      motion = MoveAxisTo(STEPPER_AXIS_Z, slot->action_z_pulses);
      if (motion == 2U) EnterState(MISSION_ACTION_GRIP);
      else if (motion == 0U) EnterFault();
      break;

    case MISSION_ACTION_GRIP:
      if (!g_stage_started)
      {
        ServoControl_SetAngle(1U, (g_kind == ACTION_KIND_PICKUP) ?
            config.gripper_closed_degrees : config.gripper_release_degrees);
        g_deadline = HAL_GetTick() + MISSION_ACTION_GRIP_HOLD_MS;
        g_stage_started = 1U;
      }
      else if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(MISSION_ACTION_RAISE);
      break;

    case MISSION_ACTION_RAISE:
      motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
      if (motion == 2U)
      {
        WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X), 1U,
                                 0, 1U);
        EnterState((g_kind == ACTION_KIND_DROP) ?
                   MISSION_ACTION_RESET_TOOL : MISSION_ACTION_DONE);
      }
      else if (motion == 0U) EnterFault();
      break;

    case MISSION_ACTION_RESET_TOOL:
      if (!g_stage_started)
      {
        ServoControl_SetAngle(0U, config.servo_initial_degrees[0]);
        ServoControl_SetAngle(1U, config.gripper_closed_degrees);
        g_deadline = HAL_GetTick() + MISSION_ACTION_SERVO_SETTLE_MS;
        g_stage_started = 1U;
      }
      else if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(MISSION_ACTION_DONE);
      break;

    default:
      break;
  }
}

void MissionAction_Abort(void)
{
  if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X)) WorldMap_InvalidateX();
  if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z)) WorldMap_InvalidateZ();
  StepperAxis_StopAll();
  EnterState(MISSION_ACTION_IDLE);
}

MissionActionState MissionAction_GetState(void)
{
  return g_state;
}

WorldSlotId MissionAction_GetSlot(void)
{
  return g_slot_id;
}
