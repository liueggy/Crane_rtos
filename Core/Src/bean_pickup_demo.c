#include "bean_pickup_demo.h"

#include "app_config.h"
#include "photo_sensor.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "world_map.h"

#define BEAN_PICKUP_DIRECTION_SETTLE_MS 300U
#define BEAN_PICKUP_X_HOME_TIMEOUT_MS  22000U
#define BEAN_PICKUP_SERVO_SETTLE_MS      600U
#define BEAN_PICKUP_GRIP_HOLD_MS         900U

/* 位置1/2/3按场地从左到右排列，并复用已经实机标定的X/Z坐标。 */
static const WorldSlotId k_pickup_slots[] = {
  WORLD_SLOT_BEAN_TOP_LEFT,
  WORLD_SLOT_BEAN_OFFSET,
  WORLD_SLOT_BEAN_TOP_RIGHT,
};

static BeanPickupDemoState g_state;
static uint8_t g_selected_position;
static uint8_t g_start_ready;
static uint8_t g_stage_started;
static uint32_t g_deadline;

static uint8_t HasSafetyFault(void)
{
  return (SafetyManager_GetFlags() &
          (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) ? 1U : 0U;
}

static void EnterState(BeanPickupDemoState state)
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

static void EnterFault(void)
{
  StopAxes();
  EnterState(BEAN_PICKUP_DEMO_FAULT);
}

static void StartPickupHome(void)
{
  if (HasSafetyFault() || StepperAxis_IsEnabled(STEPPER_AXIS_X) ||
      StepperAxis_IsEnabled(STEPPER_AXIS_Z))
  {
    EnterFault();
    return;
  }

  g_start_ready = 0U;
  /* 回零控制结束时Z位于触底位置；先停脉冲、设向上方向并等待。 */
  if ((PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U) &&
      !StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_Z, 0U))
  {
    EnterFault();
    return;
  }
  StepperAxis_SetDirectionReverse(STEPPER_AXIS_Z, 1U);
  g_deadline = HAL_GetTick() + BEAN_PICKUP_DIRECTION_SETTLE_MS;
  EnterState(BEAN_PICKUP_DEMO_HOME_Z_SETTLE);
}

static void StartPickupAction(void)
{
  const WorldSlotPose *slot = WorldMap_GetSlot(k_pickup_slots[g_selected_position]);

  if (HasSafetyFault() || !g_start_ready || (slot == 0) || !slot->calibrated ||
      StepperAxis_IsEnabled(STEPPER_AXIS_X) ||
      StepperAxis_IsEnabled(STEPPER_AXIS_Z))
  {
    EnterFault();
    return;
  }
  if (StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) != 0)
  {
    StepperAxis_SetDirectionReverse(
        STEPPER_AXIS_Z,
        (StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) > 0) ? 1U : 0U);
    g_deadline = HAL_GetTick() + BEAN_PICKUP_DIRECTION_SETTLE_MS;
    EnterState(BEAN_PICKUP_DEMO_PREPARE_Z_SETTLE);
  }
  else EnterState(BEAN_PICKUP_DEMO_OPEN_GRIPPER);
}

void BeanPickupDemo_Init(void)
{
  g_selected_position = 0U;
  g_start_ready = 0U;
  EnterState(BEAN_PICKUP_DEMO_IDLE);
}

void BeanPickupDemo_SelectPosition(uint8_t position)
{
  if ((position < 3U) && !BeanPickupDemo_IsRunning())
    g_selected_position = position;
}

void BeanPickupDemo_HandlePower(void)
{
  if (BeanPickupDemo_IsRunning()) return;
  if (!g_start_ready) StartPickupHome();
  else StartPickupAction();
}

uint8_t BeanPickupDemo_StartReferencedPickup(uint8_t position)
{
  if ((position >= 3U) || BeanPickupDemo_IsRunning() ||
      (g_state == BEAN_PICKUP_DEMO_FAULT)) return 0U;
  g_selected_position = position;
  g_start_ready = 1U;
  StartPickupAction();
  return (g_state != BEAN_PICKUP_DEMO_FAULT) ? 1U : 0U;
}

void BeanPickupDemo_Abort(void)
{
  StopAxes();
  /* 中止抓取时保持夹爪当前位置，避免已经夹住的豆子被意外释放。 */
  g_start_ready = 0U;
  EnterState(BEAN_PICKUP_DEMO_IDLE);
}

void BeanPickupDemo_Process(void)
{
  AppConfig config;
  const WorldSlotPose *slot;
  uint8_t motion;

  if (!BeanPickupDemo_IsRunning()) return;
  StepperAxis_ProcessPhotoInterlock();
  if (HasSafetyFault())
  {
    EnterFault();
    return;
  }
  slot = WorldMap_GetSlot(k_pickup_slots[g_selected_position]);
  if ((slot == 0) || !slot->calibrated)
  {
    EnterFault();
    return;
  }
  AppConfig_GetSnapshot(&config);

  switch (g_state)
  {
    case BEAN_PICKUP_DEMO_HOME_Z_SETTLE:
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(BEAN_PICKUP_DEMO_HOME_Z_TOP);
      break;

    case BEAN_PICKUP_DEMO_HOME_Z_TOP:
      motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
      if (motion == 2U)
      {
        StepperAxis_SetDirectionReverse(STEPPER_AXIS_X, 1U);
        g_deadline = HAL_GetTick() + BEAN_PICKUP_DIRECTION_SETTLE_MS;
        EnterState(BEAN_PICKUP_DEMO_HOME_X_SETTLE);
      }
      else if (motion == 0U) EnterFault();
      break;

    case BEAN_PICKUP_DEMO_HOME_X_SETTLE:
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(BEAN_PICKUP_DEMO_HOME_X_RIGHT);
      break;

    case BEAN_PICKUP_DEMO_HOME_X_RIGHT:
      if (!g_stage_started)
      {
        if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
        {
          EnterFault();
          break;
        }
        StepperAxis_SetDirectionReverse(STEPPER_AXIS_X, 1U);
        StepperAxis_SetEnabled(STEPPER_AXIS_X, 1U);
        if (!StepperAxis_IsEnabled(STEPPER_AXIS_X))
        {
          EnterFault();
          break;
        }
        g_deadline = HAL_GetTick() + BEAN_PICKUP_X_HOME_TIMEOUT_MS;
        g_stage_started = 1U;
      }
      else if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
      {
        StepperAxis_ProcessPhotoInterlock();
        if (!StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_X) ||
            !StepperAxis_SetPositionPulses(STEPPER_AXIS_X, 0))
        {
          EnterFault();
          break;
        }
        WorldMap_SetAxisPosition(0, 1U, 0, 1U);
        if (!StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_X, 1U))
        {
          EnterFault();
          break;
        }
        StepperAxis_SetDirectionReverse(STEPPER_AXIS_X, 0U);
        g_deadline = HAL_GetTick() + BEAN_PICKUP_DIRECTION_SETTLE_MS;
        EnterState(BEAN_PICKUP_DEMO_CLEAR_X_SETTLE);
      }
      else if ((int32_t)(HAL_GetTick() - g_deadline) >= 0) EnterFault();
      break;

    case BEAN_PICKUP_DEMO_CLEAR_X_SETTLE:
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(BEAN_PICKUP_DEMO_CLEAR_X_LIMIT);
      break;

    case BEAN_PICKUP_DEMO_CLEAR_X_LIMIT:
      if (!g_stage_started)
      {
        if (StepperAxis_MovePulses(STEPPER_AXIS_X,
                                   STEPPER_X_SAFE_MARGIN_PULSES, 0U) != HAL_OK)
        {
          EnterFault();
          break;
        }
        g_stage_started = 1U;
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X))
      {
        if ((StepperAxis_GetRemainingPulses(STEPPER_AXIS_X) != 0U) ||
            (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U))
        {
          EnterFault();
          break;
        }
        WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X), 1U,
                                 0, 1U);
        g_start_ready = 1U;
        EnterState(BEAN_PICKUP_DEMO_READY);
      }
      break;

    case BEAN_PICKUP_DEMO_PREPARE_Z_SETTLE:
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(BEAN_PICKUP_DEMO_PREPARE_Z);
      break;

    case BEAN_PICKUP_DEMO_PREPARE_Z:
      motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
      if (motion == 2U) EnterState(BEAN_PICKUP_DEMO_OPEN_GRIPPER);
      else if (motion == 0U) EnterFault();
      break;

    case BEAN_PICKUP_DEMO_OPEN_GRIPPER:
      if (!g_stage_started)
      {
        ServoControl_SetAngle(0U, slot->tool_yaw_degrees);
        ServoControl_SetAngle(1U, config.gripper_open_degrees);
        g_deadline = HAL_GetTick() + BEAN_PICKUP_SERVO_SETTLE_MS;
        g_stage_started = 1U;
      }
      else if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
      {
        int32_t current_x = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
        StepperAxis_SetDirectionReverse(STEPPER_AXIS_X,
            (current_x > slot->gantry_x_pulses) ? 1U : 0U);
        g_deadline = HAL_GetTick() + BEAN_PICKUP_DIRECTION_SETTLE_MS;
        EnterState(BEAN_PICKUP_DEMO_MOVE_X_SETTLE);
      }
      break;

    case BEAN_PICKUP_DEMO_MOVE_X_SETTLE:
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(BEAN_PICKUP_DEMO_MOVE_X);
      break;

    case BEAN_PICKUP_DEMO_MOVE_X:
      motion = MoveAxisTo(STEPPER_AXIS_X, slot->gantry_x_pulses);
      if (motion == 2U) EnterState(BEAN_PICKUP_DEMO_DESCEND);
      else if (motion == 0U) EnterFault();
      break;

    case BEAN_PICKUP_DEMO_DESCEND:
      motion = MoveAxisTo(STEPPER_AXIS_Z, slot->action_z_pulses);
      if (motion == 2U) EnterState(BEAN_PICKUP_DEMO_CLOSE_GRIPPER);
      else if (motion == 0U) EnterFault();
      break;

    case BEAN_PICKUP_DEMO_CLOSE_GRIPPER:
      if (!g_stage_started)
      {
        ServoControl_SetAngle(1U, config.gripper_closed_degrees);
        g_deadline = HAL_GetTick() + BEAN_PICKUP_GRIP_HOLD_MS;
        g_stage_started = 1U;
      }
      else if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
      {
        StepperAxis_SetDirectionReverse(STEPPER_AXIS_Z, 1U);
        g_deadline = HAL_GetTick() + BEAN_PICKUP_DIRECTION_SETTLE_MS;
        EnterState(BEAN_PICKUP_DEMO_RAISE_SETTLE);
      }
      break;

    case BEAN_PICKUP_DEMO_RAISE_SETTLE:
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(BEAN_PICKUP_DEMO_RAISE);
      break;

    case BEAN_PICKUP_DEMO_RAISE:
      motion = MoveAxisTo(STEPPER_AXIS_Z, 0);
      if (motion == 2U)
      {
        WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X), 1U,
                                 StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
        EnterState(BEAN_PICKUP_DEMO_COMPLETE);
      }
      else if (motion == 0U) EnterFault();
      break;

    default:
      break;
  }
}

BeanPickupDemoState BeanPickupDemo_GetState(void)
{
  return g_state;
}

uint8_t BeanPickupDemo_GetSelectedPosition(void)
{
  return g_selected_position;
}

uint8_t BeanPickupDemo_IsRunning(void)
{
  return (g_state >= BEAN_PICKUP_DEMO_HOME_Z_SETTLE) &&
         (g_state <= BEAN_PICKUP_DEMO_RAISE) &&
         (g_state != BEAN_PICKUP_DEMO_READY);
}

uint8_t BeanPickupDemo_IsStartReady(void)
{
  return g_start_ready;
}
