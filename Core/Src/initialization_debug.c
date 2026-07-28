#include "initialization_debug.h"

#include "app_config.h"
#include "chassis_motion.h"
#include "photo_sensor.h"
#include "robot_controller.h"
#include "safety_manager.h"
#include "stepper_axis.h"

#define INIT_DEBUG_Z_TIMEOUT_MS       10000U
#define INIT_DEBUG_X_HOME_TIMEOUT_MS  22000U
#define INIT_DEBUG_X_MOVE_TIMEOUT_MS  15000U
#define INIT_DEBUG_DIRECTION_SETTLE_MS  300U
#define INIT_DEBUG_X_CENTER_PULSES    (STEPPER_X_TRAVEL_PULSES / 2U)

static InitializationDebugState g_state;
static InitializationDebugState g_after_settle_state;
static uint8_t g_stage_started;
static uint32_t g_deadline;

static uint8_t HasSafetyFault(void)
{
  return (SafetyManager_GetFlags() &
          (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) ? 1U : 0U;
}

static uint8_t DeadlineExpired(void)
{
  return ((int32_t)(HAL_GetTick() - g_deadline) >= 0) ? 1U : 0U;
}

static void StopAxes(void)
{
  StepperAxis_SetEnabled(STEPPER_AXIS_X, 0U);
  StepperAxis_SetEnabled(STEPPER_AXIS_Z, 0U);
}

static void EnterState(InitializationDebugState state)
{
  g_state = state;
  g_stage_started = 0U;
}

static void EnterDirectionSettle(StepperAxisId axis, uint8_t reverse,
                                 InitializationDebugState next_state)
{
  StepperAxis_SetEnabled(axis, 0U);
  StepperAxis_SetDirectionReverse(axis, reverse);
  g_after_settle_state = next_state;
  g_deadline = HAL_GetTick() + INIT_DEBUG_DIRECTION_SETTLE_MS;
  EnterState(INITIALIZATION_DEBUG_DIRECTION_SETTLE);
}

static void EnterFault(void)
{
  StopAxes();
  EnterState(INITIALIZATION_DEBUG_FAULT);
}

static uint8_t StartContinuousMove(StepperAxisId axis, uint8_t reverse,
                                   uint32_t timeout_ms)
{
  StepperAxis_SetDirectionReverse(axis, reverse);
  StepperAxis_SetEnabled(axis, 1U);
  if (!StepperAxis_IsEnabled(axis)) return 0U;
  g_deadline = HAL_GetTick() + timeout_ms;
  g_stage_started = 1U;
  return 1U;
}

static uint8_t AcceptBottomReference(void)
{
  if (!StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_Z) &&
      !StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_Z, 0U)) return 0U;
  return StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_Z) &&
         StepperAxis_SetPositionPulses(STEPPER_AXIS_Z,
                                       (int32_t)STEPPER_Z_TRAVEL_PULSES);
}

void InitializationDebug_Init(void)
{
  g_after_settle_state = INITIALIZATION_DEBUG_IDLE;
  EnterState(INITIALIZATION_DEBUG_IDLE);
}

void InitializationDebug_Abort(void)
{
  StopAxes();
  EnterState(INITIALIZATION_DEBUG_IDLE);
}

void InitializationDebug_ToggleRunning(void)
{
  RobotState robot_state;

  if (InitializationDebug_IsRunning())
  {
    InitializationDebug_Abort();
    return;
  }

  robot_state = RobotController_GetState();
  if (HasSafetyFault() ||
      ((robot_state != ROBOT_STATE_IDLE) &&
       (robot_state != ROBOT_STATE_CALIBRATION_REQUIRED) &&
       (robot_state != ROBOT_STATE_FINISHED)))
  {
    EnterState(INITIALIZATION_DEBUG_FAULT);
    return;
  }

  ChassisMotion_StopManual();
  StopAxes();
  EnterState(INITIALIZATION_DEBUG_SEEK_Z_BOTTOM);
}

void InitializationDebug_Process(void)
{
  if (!InitializationDebug_IsRunning()) return;

  /* 与遥控任务共用同一互锁处理，确保调试任务也能及时响应PB11。 */
  StepperAxis_ProcessPhotoInterlock();
  if (HasSafetyFault())
  {
    EnterFault();
    return;
  }

  switch (g_state)
  {
    case INITIALIZATION_DEBUG_SEEK_Z_BOTTOM:
      if (!g_stage_started)
      {
        if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
        {
          /* 规定调试起姿为X中点、Z触底，因此开机遮挡归属于Z下限。 */
          if (!AcceptBottomReference()) EnterFault();
          else EnterDirectionSettle(STEPPER_AXIS_Z, 1U,
                                    INITIALIZATION_DEBUG_RAISE_Z_TOP);
        }
        else if (!StartContinuousMove(STEPPER_AXIS_Z, 0U,
                                      INIT_DEBUG_Z_TIMEOUT_MS)) EnterFault();
      }
      else if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
      {
        StepperAxis_ProcessPhotoInterlock();
        if (!AcceptBottomReference()) EnterFault();
        else EnterDirectionSettle(STEPPER_AXIS_Z, 1U,
                                  INITIALIZATION_DEBUG_RAISE_Z_TOP);
      }
      else if (DeadlineExpired()) EnterFault();
      break;

    case INITIALIZATION_DEBUG_DIRECTION_SETTLE:
      if (DeadlineExpired()) EnterState(g_after_settle_state);
      break;

    case INITIALIZATION_DEBUG_RAISE_Z_TOP:
      if (!g_stage_started)
      {
        if (StepperAxis_MovePulses(STEPPER_AXIS_Z,
                                   STEPPER_Z_TRAVEL_PULSES, 1U) != HAL_OK)
        {
          EnterFault();
          break;
        }
        g_deadline = HAL_GetTick() + INIT_DEBUG_Z_TIMEOUT_MS;
        g_stage_started = 1U;
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z))
      {
        if ((StepperAxis_GetRemainingPulses(STEPPER_AXIS_Z) != 0U) ||
            !StepperAxis_SetPositionPulses(STEPPER_AXIS_Z, 0)) EnterFault();
        else EnterDirectionSettle(STEPPER_AXIS_X, 1U,
                                  INITIALIZATION_DEBUG_HOME_X_RIGHT);
      }
      else if (DeadlineExpired()) EnterFault();
      break;

    case INITIALIZATION_DEBUG_HOME_X_RIGHT:
      if (!g_stage_started)
      {
        if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U ||
            !StartContinuousMove(STEPPER_AXIS_X, 1U,
                                 INIT_DEBUG_X_HOME_TIMEOUT_MS)) EnterFault();
      }
      else if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
      {
        StepperAxis_ProcessPhotoInterlock();
        if (!StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_X) ||
            !StepperAxis_SetPositionPulses(STEPPER_AXIS_X, 0)) EnterFault();
        else EnterDirectionSettle(STEPPER_AXIS_X, 0U,
                                  INITIALIZATION_DEBUG_MOVE_X_CENTER);
      }
      else if (DeadlineExpired()) EnterFault();
      break;

    case INITIALIZATION_DEBUG_MOVE_X_CENTER:
      if (!g_stage_started)
      {
        if (!StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_X, 1U) ||
            (StepperAxis_MovePulses(STEPPER_AXIS_X,
                                    INIT_DEBUG_X_CENTER_PULSES, 0U) != HAL_OK))
        {
          EnterFault();
          break;
        }
        g_deadline = HAL_GetTick() + INIT_DEBUG_X_MOVE_TIMEOUT_MS;
        g_stage_started = 1U;
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X))
      {
        if ((StepperAxis_GetRemainingPulses(STEPPER_AXIS_X) != 0U) ||
            !StepperAxis_SetPositionPulses(STEPPER_AXIS_X,
                                           (int32_t)INIT_DEBUG_X_CENTER_PULSES))
          EnterFault();
        else EnterDirectionSettle(STEPPER_AXIS_Z, 0U,
                                  INITIALIZATION_DEBUG_LOWER_Z_BOTTOM);
      }
      else if (DeadlineExpired()) EnterFault();
      break;

    case INITIALIZATION_DEBUG_LOWER_Z_BOTTOM:
      if (!g_stage_started)
      {
        if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U ||
            !StartContinuousMove(STEPPER_AXIS_Z, 0U,
                                 INIT_DEBUG_Z_TIMEOUT_MS)) EnterFault();
      }
      else if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
      {
        StepperAxis_ProcessPhotoInterlock();
        if (!AcceptBottomReference()) EnterFault();
        else EnterState(INITIALIZATION_DEBUG_COMPLETE);
      }
      else if (DeadlineExpired()) EnterFault();
      break;

    default:
      break;
  }
}

InitializationDebugState InitializationDebug_GetState(void)
{
  return g_state;
}

uint8_t InitializationDebug_IsRunning(void)
{
  return (g_state >= INITIALIZATION_DEBUG_SEEK_Z_BOTTOM) &&
         (g_state <= INITIALIZATION_DEBUG_LOWER_Z_BOTTOM);
}
