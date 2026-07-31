#include "mission_action.h"

#include "app_config.h"
#include "photo_sensor.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"

#define MISSION_ACTION_SERVO_SETTLE_MS 600U
#define MISSION_ACTION_GRIP_HOLD_MS    900U
#define MISSION_ACTION_LOADED_YAW_DPS   50U
#define MISSION_ACTION_EMPTY_YAW_DPS   100U
#define MISSION_ACTION_YAW_SETTLE_MS   200U
#define MISSION_ACTION_BUSY_RETRY_MS    50U

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
static uint8_t g_retry_used;
static uint32_t g_retry_tick;
static MissionActionFaultCode g_fault;

static void EnterState(MissionActionState state)
{
  g_state = state;
  g_stage_started = 0U;
  g_retry_used = 0U;
  g_retry_tick = 0U;
}

static void EnterFault(MissionActionFaultCode fault)
{
  if ((g_state == MISSION_ACTION_MOVE_X) ||
      StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X)) WorldMap_InvalidateX();
  if ((g_state == MISSION_ACTION_LIFT) ||
      (g_state == MISSION_ACTION_DESCEND) ||
      (g_state == MISSION_ACTION_RAISE) ||
      StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z)) WorldMap_InvalidateZ();
  ServoControl_CancelSlew(0U);
  ServoControl_CancelSlew(1U);
  StepperAxis_StopMotionPreserveZ();
  g_fault = fault;
  EnterState(MISSION_ACTION_FAULT);
}

/* 0失败，1运动中，2已到位，3为不改变坐标的瞬态忙。 */
static uint8_t MoveAxisTo(StepperAxisId axis, int32_t target)
{
  int32_t current = StepperAxis_GetPositionPulses(axis);
  uint32_t pulses;
  HAL_StatusTypeDef status;
  if (StepperAxis_IsPulseMoveActive(axis)) return 1U;
  if (current == target) return 2U;
  if ((target < 0) ||
      ((axis == STEPPER_AXIS_X) &&
       (target > (int32_t)STEPPER_X_TRAVEL_PULSES)) ||
      ((axis == STEPPER_AXIS_Z) &&
       (target > (int32_t)STEPPER_Z_TRAVEL_PULSES))) return 0U;
  pulses = (uint32_t)((current > target) ? current - target : target - current);
  status = StepperAxis_MovePulses(axis, pulses,
                                  (current > target) ? 1U : 0U);
  if (status == HAL_OK) return 1U;
  return (status == HAL_BUSY) ? 3U : 0U;
}

static uint8_t WaitForOneBusyRetry(MissionActionFaultCode fault)
{
  if (!g_retry_used)
  {
    g_retry_used = 1U;
    g_retry_tick = HAL_GetTick() + MISSION_ACTION_BUSY_RETRY_MS;
    return 1U;
  }
  if ((int32_t)(HAL_GetTick() - g_retry_tick) < 0) return 1U;
  EnterFault(fault);
  return 0U;
}

static uint32_t SlewTimeoutMs(uint16_t target, uint16_t rate)
{
  uint16_t current = ServoControl_GetAngle(0U);
  uint32_t delta = (current > target) ? current - target : target - current;
  return ((delta * 1000U + rate - 1U) / rate) + 1000U;
}

/*
 * 数字区物理1/5号侧箱分别使用X两端挡片作为投放横移基准。
 * PB11确认归属于X轴后，直接把当前端点重建为对应的绝对坐标，
 * 避免限位已经停车但剩余脉冲尚未归零时被误判为动作失败。
 */
static uint8_t ConfirmSideDropXReference(const WorldSlotPose *slot)
{
  const WorldPose *pose;
  int32_t endpoint;

  if ((g_kind != ACTION_KIND_DROP) || (g_state != MISSION_ACTION_MOVE_X) ||
      (slot == 0) ||
      ((g_slot_id != WORLD_SLOT_NUMBER_OFFSET_RIGHT) &&
       (g_slot_id != WORLD_SLOT_NUMBER_OFFSET_LEFT)) ||
      !StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_X)) return 0U;

  endpoint = (g_slot_id == WORLD_SLOT_NUMBER_OFFSET_RIGHT) ?
             0 : (int32_t)STEPPER_X_TRAVEL_PULSES;
  if ((slot->gantry_x_pulses != endpoint) ||
      StepperAxis_IsEnabled(STEPPER_AXIS_X) ||
      !StepperAxis_SetPositionPulses(STEPPER_AXIS_X, endpoint)) return 0U;

  pose = WorldMap_GetPose();
  WorldMap_SetAxisPosition(endpoint, 1U,
                           StepperAxis_GetPositionPulses(STEPPER_AXIS_Z),
                           (pose != 0) ? pose->z_valid : 0U);
  return 1U;
}

static uint8_t IsSideNumberDrop(void)
{
  return (g_kind == ACTION_KIND_DROP) &&
         ((g_slot_id == WORLD_SLOT_NUMBER_OFFSET_RIGHT) ||
          (g_slot_id == WORLD_SLOT_NUMBER_OFFSET_LEFT));
}

/*
 * 侧箱X挡片在整个投放过程中持续遮挡PB11。Z仍使用绝对脉冲到3609，
 * 因此在Z启动前显式转交共用光电门归属，并把当前运动方向设为允许方向。
 */
static uint8_t PrepareSideDropZMotion(uint8_t z_reverse)
{
  if (!IsSideNumberDrop() ||
      (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) == 0U) ||
      StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z)) return 1U;

  return StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_Z,
                                         z_reverse ? 0U : 1U);
}

/* 投放抬顶后把PB11交还X轴，只允许从当前端部挡片向场内退出。 */
static uint8_t PrepareSideDropXEscape(void)
{
  uint8_t blocked_reverse;
  if (!IsSideNumberDrop() ||
      (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) == 0U)) return 1U;

  blocked_reverse = (g_slot_id == WORLD_SLOT_NUMBER_OFFSET_RIGHT) ? 1U : 0U;
  return StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_X, blocked_reverse);
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
  g_fault = MISSION_ACTION_FAULT_NONE;
  EnterState(MISSION_ACTION_LIFT);
  return 1U;
}

void MissionAction_Init(void)
{
  g_kind = ACTION_KIND_PICKUP;
  g_slot_id = WORLD_SLOT_BEAN_TOP_LEFT;
  g_fault = MISSION_ACTION_FAULT_NONE;
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
    EnterFault(MISSION_ACTION_FAULT_SAFETY);
    return;
  }
  ServoControl_Process();
  StepperAxis_ProcessPhotoInterlock();
  slot = WorldMap_GetSlot(g_slot_id);
  if ((slot == 0) || !slot->calibrated)
  {
    EnterFault(MISSION_ACTION_FAULT_SLOT);
    return;
  }
  AppConfig_GetSnapshot(&config);

  switch (g_state)
  {
    case MISSION_ACTION_LIFT:
      motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
      if (motion == 2U)
      {
        if (StepperAxis_GetRemainingPulses(STEPPER_AXIS_Z) != 0U)
          EnterFault(MISSION_ACTION_FAULT_PULSE_INCOMPLETE);
        else EnterState(MISSION_ACTION_PREPARE_TOOL);
      }
      else if (motion == 3U)
        (void)WaitForOneBusyRetry(MISSION_ACTION_FAULT_Z_TOP);
      else if (motion == 0U) EnterFault(MISSION_ACTION_FAULT_Z_TOP);
      break;

    case MISSION_ACTION_PREPARE_TOOL:
      if (!g_stage_started)
      {
        if (g_kind == ACTION_KIND_PICKUP)
        {
          ServoControl_SetAngle(0U, slot->tool_yaw_degrees);
          ServoControl_SetAngle(1U, config.gripper_open_degrees);
          g_deadline = HAL_GetTick() + MISSION_ACTION_SERVO_SETTLE_MS;
        }
        else
        {
          if (!ServoControl_StartSlew(0U, slot->tool_yaw_degrees,
                                      MISSION_ACTION_LOADED_YAW_DPS))
          {
            EnterFault(MISSION_ACTION_FAULT_SERVO_TIMEOUT);
            break;
          }
          g_deadline = HAL_GetTick() +
                       SlewTimeoutMs(slot->tool_yaw_degrees,
                                     MISSION_ACTION_LOADED_YAW_DPS);
        }
        g_stage_started = 1U;
      }
      else if (g_kind == ACTION_KIND_PICKUP)
      {
        if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
          EnterState(MISSION_ACTION_MOVE_X);
      }
      else if (g_stage_started == 1U)
      {
        if (ServoControl_IsSlewActive(0U))
        {
          if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
            EnterFault(MISSION_ACTION_FAULT_SERVO_TIMEOUT);
        }
        else
        {
          g_stage_started = 2U;
          g_deadline = HAL_GetTick() + MISSION_ACTION_YAW_SETTLE_MS;
        }
      }
      else if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(MISSION_ACTION_MOVE_X);
      break;

    case MISSION_ACTION_MOVE_X:
      if (ConfirmSideDropXReference(slot))
        EnterState(MISSION_ACTION_DESCEND);
      else
      {
        motion = MoveAxisTo(STEPPER_AXIS_X, slot->gantry_x_pulses);
        if (motion == 2U)
        {
          if (StepperAxis_GetRemainingPulses(STEPPER_AXIS_X) != 0U)
            EnterFault(MISSION_ACTION_FAULT_PULSE_INCOMPLETE);
          else EnterState(MISSION_ACTION_DESCEND);
        }
        else if (motion == 3U)
          (void)WaitForOneBusyRetry(MISSION_ACTION_FAULT_X_MOVE);
        else if (motion == 0U) EnterFault(MISSION_ACTION_FAULT_X_MOVE);
      }
      break;

    case MISSION_ACTION_DESCEND:
      if (!PrepareSideDropZMotion(0U))
        (void)WaitForOneBusyRetry(MISSION_ACTION_FAULT_PB11_OWNER);
      else
      {
        motion = MoveAxisTo(STEPPER_AXIS_Z, slot->action_z_pulses);
        if (motion == 2U) EnterState(MISSION_ACTION_GRIP);
        else if (motion == 3U)
          (void)WaitForOneBusyRetry(MISSION_ACTION_FAULT_Z_MOVE);
        else if (motion == 0U) EnterFault(MISSION_ACTION_FAULT_Z_MOVE);
      }
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
      if (!PrepareSideDropZMotion(1U))
        (void)WaitForOneBusyRetry(MISSION_ACTION_FAULT_PB11_OWNER);
      else
      {
        motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
        if (motion == 2U)
        {
          WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X), 1U,
                                   0, 1U);
          if (StepperAxis_GetRemainingPulses(STEPPER_AXIS_Z) != 0U)
            EnterFault(MISSION_ACTION_FAULT_PULSE_INCOMPLETE);
          else if (!PrepareSideDropXEscape())
            EnterFault(MISSION_ACTION_FAULT_PB11_OWNER);
          else EnterState((g_kind == ACTION_KIND_DROP) ?
                          MISSION_ACTION_RESET_TOOL : MISSION_ACTION_DONE);
        }
        else if (motion == 3U)
          (void)WaitForOneBusyRetry(MISSION_ACTION_FAULT_Z_TOP);
        else if (motion == 0U) EnterFault(MISSION_ACTION_FAULT_Z_TOP);
      }
      break;

    case MISSION_ACTION_RESET_TOOL:
      if (!g_stage_started)
      {
        ServoControl_SetAngle(1U, config.gripper_closed_degrees);
        if (!ServoControl_StartSlew(0U, config.servo_initial_degrees[0],
                                    MISSION_ACTION_EMPTY_YAW_DPS))
        {
          EnterFault(MISSION_ACTION_FAULT_SERVO_TIMEOUT);
          break;
        }
        g_deadline = HAL_GetTick() +
                     SlewTimeoutMs(config.servo_initial_degrees[0],
                                   MISSION_ACTION_EMPTY_YAW_DPS);
        g_stage_started = 1U;
      }
      else if (g_stage_started == 1U)
      {
        if (ServoControl_IsSlewActive(0U))
        {
          if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
            EnterFault(MISSION_ACTION_FAULT_SERVO_TIMEOUT);
        }
        else
        {
          g_stage_started = 2U;
          g_deadline = HAL_GetTick() + MISSION_ACTION_YAW_SETTLE_MS;
        }
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
  ServoControl_CancelSlew(0U);
  ServoControl_CancelSlew(1U);
  StepperAxis_StopMotionPreserveZ();
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

MissionActionFaultCode MissionAction_GetFaultCode(void)
{
  return g_fault;
}
