#include "bean_sequence_demo.h"

#include "app_config.h"
#include "bean_pickup_demo.h"
#include "chassis_motion.h"
#include "photo_sensor.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "world_map.h"

#define BEAN_SEQUENCE_CHASSIS_RPM              50U
#define BEAN_SEQUENCE_START_TO_B_MM          1796
#define BEAN_SEQUENCE_START_TO_B_LANDMARKS      3U
#define BEAN_SEQUENCE_B_TO_AC_MM              257
#define BEAN_SEQUENCE_START_TO_B_TIMEOUT_MS  15000U
#define BEAN_SEQUENCE_B_TO_AC_TIMEOUT_MS      6500U
#define BEAN_SEQUENCE_RELEASE_SETTLE_MS        600U

/* 抓豆控制页面位置编号：0=A(左)，1=B(中间凸出)，2=C(右)。 */
#define BEAN_POSITION_A 0U
#define BEAN_POSITION_B 1U
#define BEAN_POSITION_C 2U

static BeanSequenceDemoState g_state;
static uint8_t g_stage_started;
static uint32_t g_deadline;

static uint8_t HasSafetyFault(void)
{
  return (SafetyManager_GetFlags() &
          (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) ? 1U : 0U;
}

static void EnterState(BeanSequenceDemoState state)
{
  g_state = state;
  g_stage_started = 0U;
}

static void EnterFault(void)
{
  ChassisMotion_Stop();
  BeanPickupDemo_Abort();
  StepperAxis_StopAll();
  EnterState(BEAN_SEQUENCE_DEMO_FAULT);
}

static uint8_t PickupCompleted(void)
{
  if (BeanPickupDemo_GetState() == BEAN_PICKUP_DEMO_FAULT)
  {
    EnterFault();
    return 0U;
  }
  return (BeanPickupDemo_GetState() == BEAN_PICKUP_DEMO_COMPLETE) ? 1U : 0U;
}

static uint8_t StartPickup(uint8_t position)
{
  return BeanPickupDemo_StartReferencedPickup(position);
}

static void StartRelease(void)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);
  /* 在当前箱正上方释放回原箱，避免把上一箱豆子带到下一箱。 */
  ServoControl_SetAngle(1U, config.gripper_open_degrees);
  g_deadline = HAL_GetTick() + BEAN_SEQUENCE_RELEASE_SETTLE_MS;
}

void BeanSequenceDemo_Init(void)
{
  EnterState(BEAN_SEQUENCE_DEMO_IDLE);
}

void BeanSequenceDemo_Start(void)
{
  if (g_state != BEAN_SEQUENCE_DEMO_IDLE) return;
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
  /* 起点规定姿态：X在横梁中点，Z触底且PB11被Z挡片遮挡。 */
  (void)StepperAxis_SetPositionPulses(STEPPER_AXIS_X,
                                     (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U));
  (void)StepperAxis_SetPositionPulses(STEPPER_AXIS_Z,
                                     (int32_t)STEPPER_Z_TRAVEL_PULSES);
  WorldMap_SetPoseAtStart();
  BeanPickupDemo_HandlePower();
  EnterState(BEAN_SEQUENCE_DEMO_REFERENCE);
}

void BeanSequenceDemo_Abort(void)
{
  ChassisMotion_Stop();
  BeanPickupDemo_Abort();
  StepperAxis_StopAll();
  EnterState(BEAN_SEQUENCE_DEMO_IDLE);
}

void BeanSequenceDemo_Process(void)
{
  if (!BeanSequenceDemo_IsRunning()) return;
  if (HasSafetyFault())
  {
    EnterFault();
    return;
  }

  switch (g_state)
  {
    case BEAN_SEQUENCE_DEMO_REFERENCE:
      if (BeanPickupDemo_GetState() == BEAN_PICKUP_DEMO_FAULT) EnterFault();
      else if (BeanPickupDemo_IsStartReady())
        EnterState(BEAN_SEQUENCE_DEMO_MOVE_TO_B);
      break;

    case BEAN_SEQUENCE_DEMO_MOVE_TO_B:
      if (!g_stage_started)
      {
        g_stage_started = ChassisMotion_StartRouteThroughLandmarks(
            -BEAN_SEQUENCE_START_TO_B_MM,
            BEAN_SEQUENCE_START_TO_B_LANDMARKS,
            BEAN_SEQUENCE_CHASSIS_RPM,
            BEAN_SEQUENCE_START_TO_B_TIMEOUT_MS);
        if (!g_stage_started) EnterFault();
      }
      else if (ChassisMotion_DidRouteSegmentFail()) EnterFault();
      else if (ChassisMotion_IsRouteSegmentDone())
      {
        WorldMap_SetKnownStation(WORLD_STATION_BEAN_B_PICK);
        if (!StartPickup(BEAN_POSITION_B)) EnterFault();
        else EnterState(BEAN_SEQUENCE_DEMO_PICK_B);
      }
      break;

    case BEAN_SEQUENCE_DEMO_PICK_B:
      if (PickupCompleted())
      {
        StartRelease();
        EnterState(BEAN_SEQUENCE_DEMO_RELEASE_B);
      }
      break;

    case BEAN_SEQUENCE_DEMO_RELEASE_B:
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(BEAN_SEQUENCE_DEMO_MOVE_TO_AC);
      break;

    case BEAN_SEQUENCE_DEMO_MOVE_TO_AC:
      if (!g_stage_started)
      {
        g_stage_started = ChassisMotion_StartRouteThroughLandmarks(
            -BEAN_SEQUENCE_B_TO_AC_MM, 1U, BEAN_SEQUENCE_CHASSIS_RPM,
            BEAN_SEQUENCE_B_TO_AC_TIMEOUT_MS);
        if (!g_stage_started) EnterFault();
      }
      else if (ChassisMotion_DidRouteSegmentFail()) EnterFault();
      else if (ChassisMotion_IsRouteSegmentDone())
      {
        WorldMap_SetKnownStation(WORLD_STATION_BEAN_AC_PICK);
        if (!StartPickup(BEAN_POSITION_C)) EnterFault();
        else EnterState(BEAN_SEQUENCE_DEMO_PICK_C);
      }
      break;

    case BEAN_SEQUENCE_DEMO_PICK_C:
      if (PickupCompleted())
      {
        StartRelease();
        EnterState(BEAN_SEQUENCE_DEMO_RELEASE_C);
      }
      break;

    case BEAN_SEQUENCE_DEMO_RELEASE_C:
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
      {
        if (!StartPickup(BEAN_POSITION_A)) EnterFault();
        else EnterState(BEAN_SEQUENCE_DEMO_PICK_A);
      }
      break;

    case BEAN_SEQUENCE_DEMO_PICK_A:
      if (PickupCompleted()) EnterState(BEAN_SEQUENCE_DEMO_COMPLETE);
      break;

    default:
      break;
  }
}

BeanSequenceDemoState BeanSequenceDemo_GetState(void)
{
  return g_state;
}

uint8_t BeanSequenceDemo_IsRunning(void)
{
  return (g_state >= BEAN_SEQUENCE_DEMO_REFERENCE) &&
         (g_state <= BEAN_SEQUENCE_DEMO_PICK_A);
}
