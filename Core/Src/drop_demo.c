#include "drop_demo.h"

#include "app_config.h"
#include "photo_sensor.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "world_map.h"

#define DROP_DEMO_SERVO_SETTLE_MS 600U
#define DROP_DEMO_RELEASE_HOLD_MS 900U

static const WorldSlotId k_demo_slots[] = {
  WORLD_SLOT_NUMBER_BOTTOM_LEFT,
  WORLD_SLOT_NUMBER_BOTTOM_CENTER,
  WORLD_SLOT_NUMBER_BOTTOM_RIGHT,
};

static DropDemoState g_state;
static DropDemoState g_resume_state;
static uint8_t g_slot_index;
static uint8_t g_stage_started;
static uint32_t g_deadline;

static uint8_t HasSafetyFault(void)
{
  return (SafetyManager_GetFlags() &
          (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) ? 1U : 0U;
}

static void SetToolInitial(void)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);
  ServoControl_SetAngle(0U, config.servo_initial_degrees[0]);
  ServoControl_SetAngle(1U, config.gripper_closed_degrees);
}

static void SetGripperRelease(void)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);
  ServoControl_SetAngle(1U, config.gripper_release_degrees);
}

static void EnterState(DropDemoState state)
{
  g_state = state;
  g_stage_started = 0U;
}

static void StopAxes(void)
{
  StepperAxis_SetEnabled(STEPPER_AXIS_X, 0U);
  StepperAxis_SetEnabled(STEPPER_AXIS_Z, 0U);
}

/* 返回0表示无法启动，1表示运动中，2表示已经到位。 */
static uint8_t MoveAxisTo(StepperAxisId axis, int32_t target)
{
  int32_t current = StepperAxis_GetPositionPulses(axis);
  uint32_t pulses;
  uint8_t reverse;

  if (StepperAxis_IsPulseMoveActive(axis)) return 1U;
  if (current == target) return 2U;
  reverse = (current > target) ? 1U : 0U;
  pulses = (uint32_t)((current > target) ? (current - target) : (target - current));
  return (StepperAxis_MovePulses(axis, pulses, reverse) == HAL_OK) ? 1U : 0U;
}

void DropDemo_Init(void)
{
  g_slot_index = 0U;
  g_resume_state = DROP_DEMO_IDLE;
  EnterState(DROP_DEMO_IDLE);
}

void DropDemo_Abort(void)
{
  StopAxes();
  SetToolInitial();
  g_slot_index = 0U;
  EnterState(DROP_DEMO_IDLE);
}

void DropDemo_ToggleRunning(void)
{
  if (g_state == DROP_DEMO_PAUSED)
  {
    EnterState(g_resume_state);
    return;
  }
  if (DropDemo_IsRunning())
  {
    g_resume_state = g_state;
    StopAxes();
    EnterState(DROP_DEMO_PAUSED);
    return;
  }
  /* Demo允许人工保证起点：X位于右端、Z位于最高点，启动时直接建立坐标。 */
  if (StepperAxis_IsEnabled(STEPPER_AXIS_X) ||
      StepperAxis_IsEnabled(STEPPER_AXIS_Z) ||
      !StepperAxis_SetPositionPulses(STEPPER_AXIS_X, 0) ||
      !StepperAxis_SetPositionPulses(STEPPER_AXIS_Z, 0))
  {
    EnterState(DROP_DEMO_FAULT);
    return;
  }
  if ((PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U) &&
      !StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_X, 1U))
  {
    EnterState(DROP_DEMO_FAULT);
    return;
  }
  if (HasSafetyFault())
  {
    EnterState(DROP_DEMO_FAULT);
    return;
  }

  g_slot_index = 0U;
  SetToolInitial();
  EnterState(DROP_DEMO_LIFT_SAFE);
}

void DropDemo_Process(void)
{
  const WorldSlotPose *slot;
  uint8_t motion;

  if (!DropDemo_IsRunning()) return;
  if (HasSafetyFault())
  {
    StopAxes();
    EnterState(DROP_DEMO_FAULT);
    return;
  }
  slot = WorldMap_GetSlot(k_demo_slots[g_slot_index]);
  if ((slot == 0) || !slot->calibrated)
  {
    EnterState(DROP_DEMO_FAULT);
    return;
  }

  switch (g_state)
  {
    case DROP_DEMO_LIFT_SAFE:
      motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
      if (motion == 2U) EnterState(DROP_DEMO_MOVE_X);
      else if (motion == 0U) EnterState(DROP_DEMO_FAULT);
      break;

    case DROP_DEMO_MOVE_X:
      motion = MoveAxisTo(STEPPER_AXIS_X, slot->gantry_x_pulses);
      if (motion == 2U) EnterState(DROP_DEMO_ROTATE);
      else if (motion == 0U) EnterState(DROP_DEMO_FAULT);
      break;

    case DROP_DEMO_ROTATE:
      if (!g_stage_started)
      {
        ServoControl_SetAngle(0U, slot->tool_yaw_degrees);
        g_deadline = HAL_GetTick() + DROP_DEMO_SERVO_SETTLE_MS;
        g_stage_started = 1U;
      }
      else if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(DROP_DEMO_DESCEND);
      break;

    case DROP_DEMO_DESCEND:
      motion = MoveAxisTo(STEPPER_AXIS_Z, slot->action_z_pulses);
      if (motion == 2U) EnterState(DROP_DEMO_RELEASE);
      else if (motion == 0U) EnterState(DROP_DEMO_FAULT);
      break;

    case DROP_DEMO_RELEASE:
      if (!g_stage_started)
      {
        SetGripperRelease();
        g_deadline = HAL_GetTick() + DROP_DEMO_RELEASE_HOLD_MS;
        g_stage_started = 1U;
      }
      else if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(DROP_DEMO_RAISE);
      break;

    case DROP_DEMO_RAISE:
      motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
      if (motion == 2U) EnterState(DROP_DEMO_RESET_TOOL);
      else if (motion == 0U) EnterState(DROP_DEMO_FAULT);
      break;

    case DROP_DEMO_RESET_TOOL:
      if (!g_stage_started)
      {
        SetToolInitial();
        g_deadline = HAL_GetTick() + DROP_DEMO_SERVO_SETTLE_MS;
        g_stage_started = 1U;
      }
      else if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
      {
        if (++g_slot_index >= (uint8_t)(sizeof(k_demo_slots) / sizeof(k_demo_slots[0])))
        {
          g_slot_index = 2U;
          EnterState(DROP_DEMO_COMPLETE);
        }
        else EnterState(DROP_DEMO_MOVE_X);
      }
      break;

    default:
      break;
  }
}

DropDemoState DropDemo_GetState(void)
{
  return g_state;
}

uint8_t DropDemo_GetSlotNumber(void)
{
  return (uint8_t)(g_slot_index + 1U);
}

uint8_t DropDemo_IsRunning(void)
{
  return (g_state >= DROP_DEMO_LIFT_SAFE) &&
         (g_state <= DROP_DEMO_RESET_TOOL) &&
         (g_state != DROP_DEMO_PAUSED);
}
