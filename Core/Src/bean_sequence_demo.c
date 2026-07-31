#include "bean_sequence_demo.h"

#include "app_config.h"
#include "bean_pickup_demo.h"
#include "chassis_motion.h"
#include "photo_sensor.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "world_map.h"

#define BEAN_SEQUENCE_NOMINAL_RPM              50U
#define BEAN_SEQUENCE_AC_ALIGN_RPM              10U
#define BEAN_SEQUENCE_AC_ALIGN_TIMEOUT_MS      5000U
#define BEAN_SEQUENCE_START_TO_B_MM           1796
#define BEAN_SEQUENCE_START_TO_B_LANDMARKS       3U
#define BEAN_SEQUENCE_START_TO_B_TIMEOUT_MS   15000U
#define BEAN_SEQUENCE_B_TO_AC_MM               257
#define BEAN_SEQUENCE_B_TO_AC_LANDMARKS          1U
#define BEAN_SEQUENCE_B_TO_AC_TIMEOUT_MS       6500U
#define BEAN_SEQUENCE_Z_MOVE_TIMEOUT_MS      12000U
#define BEAN_SEQUENCE_GRIP_TEST_HOLD_MS        4000U
#define BEAN_SEQUENCE_RELEASE_SETTLE_MS        600U
#define BEAN_SEQUENCE_RELEASE_LIFT_MM_X10       500U
#define BEAN_SEQUENCE_RELEASE_LIFT_PULSES \
  ((STEPPER_Z_TRAVEL_PULSES * BEAN_SEQUENCE_RELEASE_LIFT_MM_X10 + \
    STEPPER_Z_TRAVEL_MM_X10 / 2U) / STEPPER_Z_TRAVEL_MM_X10)

/* 抓豆控制页面位置编号：0=A(左)，1=B(中间凸出)，2=C(右)。 */
#define BEAN_POSITION_A 0U
#define BEAN_POSITION_B 1U
#define BEAN_POSITION_C 2U

static const WorldSlotId k_sequence_slots[] = {
  WORLD_SLOT_BEAN_TOP_LEFT,
  WORLD_SLOT_BEAN_OFFSET,
  WORLD_SLOT_BEAN_TOP_RIGHT,
};

static BeanSequenceDemoState g_state;
static uint8_t g_stage_started;
static uint8_t g_current_position;
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
  StepperAxis_StopMotionPreserveZ();
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
  g_current_position = position;
  return BeanPickupDemo_StartReferencedPickup(position);
}

static uint16_t RouteSpeedRpm(void)
{
  int16_t rpm = ChassisMotion_GetTargetRpm();
  return (rpm > 0) ? (uint16_t)rpm : BEAN_SEQUENCE_NOMINAL_RPM;
}

static uint16_t ScaledRouteTimeout(uint16_t nominal_timeout_ms)
{
  uint32_t scaled = ((uint32_t)nominal_timeout_ms *
                     BEAN_SEQUENCE_NOMINAL_RPM) / RouteSpeedRpm();
  if (scaled < 100U) scaled = 100U;
  if (scaled > UINT16_MAX) scaled = UINT16_MAX;
  return (uint16_t)scaled;
}

static void StartRelease(void)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);
  /* 在当前箱正上方释放回原箱，避免把上一箱豆子带到下一箱。 */
  ServoControl_SetAngle(1U, config.gripper_open_degrees);
  g_deadline = HAL_GetTick() + BEAN_SEQUENCE_RELEASE_SETTLE_MS;
}

static uint8_t StartZMoveTo(int32_t target)
{
  int32_t current = StepperAxis_GetPositionPulses(STEPPER_AXIS_Z);
  uint32_t pulses;
  uint8_t reverse;
  if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z)) return 0U;
  if (current == target) return 2U;
  reverse = (current > target) ? 1U : 0U;
  pulses = (uint32_t)((current > target) ? current - target : target - current);
  return (StepperAxis_MovePulses(STEPPER_AXIS_Z, pulses, reverse) == HAL_OK) ? 1U : 0U;
}

static uint8_t ZMoveFinishedAt(int32_t target)
{
  if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z)) return 0U;
  return (StepperAxis_GetRemainingPulses(STEPPER_AXIS_Z) == 0U) &&
         (StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) == target);
}

static int32_t ReleaseZForSlot(const WorldSlotPose *slot)
{
  int32_t release_z = slot->action_z_pulses -
                      (int32_t)BEAN_SEQUENCE_RELEASE_LIFT_PULSES;
  return (release_z > 0) ? release_z : 0;
}

static void BeginGripHold(uint8_t position)
{
  g_current_position = position;
  g_deadline = HAL_GetTick() + BEAN_SEQUENCE_GRIP_TEST_HOLD_MS;
  EnterState(BEAN_SEQUENCE_DEMO_HOLD);
}

void BeanSequenceDemo_Init(void)
{
  g_current_position = BEAN_POSITION_B;
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
  g_current_position = BEAN_POSITION_B;
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
  StepperAxis_StopMotionPreserveZ();
  EnterState(BEAN_SEQUENCE_DEMO_IDLE);
}

void BeanSequenceDemo_Process(void)
{
  if (!BeanSequenceDemo_IsRunning()) return;
  StepperAxis_ProcessPhotoInterlock();
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
        g_stage_started = ChassisMotion_StartAlignedRouteThroughLandmarks(
            -BEAN_SEQUENCE_START_TO_B_MM,
            BEAN_SEQUENCE_START_TO_B_LANDMARKS,
            RouteSpeedRpm(),
            ScaledRouteTimeout(BEAN_SEQUENCE_START_TO_B_TIMEOUT_MS));
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

    case BEAN_SEQUENCE_DEMO_MOVE_TO_AC:
      if (!g_stage_started)
      {
        g_stage_started = ChassisMotion_StartRouteThroughLandmarks(
            -BEAN_SEQUENCE_B_TO_AC_MM,
            BEAN_SEQUENCE_B_TO_AC_LANDMARKS,
            RouteSpeedRpm(),
            ScaledRouteTimeout(BEAN_SEQUENCE_B_TO_AC_TIMEOUT_MS));
        if (!g_stage_started) EnterFault();
      }
      else if (ChassisMotion_DidRouteSegmentFail()) EnterFault();
      else if (ChassisMotion_IsRouteSegmentDone())
      {
        WorldMap_SetKnownStation(WORLD_STATION_BEAN_AC_PICK);
        EnterState(BEAN_SEQUENCE_DEMO_ALIGN_AC);
      }
      break;

    case BEAN_SEQUENCE_DEMO_ALIGN_AC:
      if ((PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_ORIGIN_SIDE) != 0U) &&
          (PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_FAR_SIDE) != 0U))
      {
        ChassisMotion_Stop();
        WorldMap_SetKnownStation(WORLD_STATION_BEAN_AC_PICK);
        if (!StartPickup(BEAN_POSITION_C)) EnterFault();
        else EnterState(BEAN_SEQUENCE_DEMO_PICK_C);
      }
      else if (!g_stage_started)
      {
        /* 原路线向豆子区为reverse=1；惯性越过后以相反方向回退。 */
        g_stage_started = ChassisMotion_StartPhotoBlockedAlignment(
            0U, BEAN_SEQUENCE_AC_ALIGN_RPM,
            BEAN_SEQUENCE_AC_ALIGN_TIMEOUT_MS);
        if (!g_stage_started) EnterFault();
      }
      else if (ChassisMotion_DidRouteSegmentFail()) EnterFault();
      else if (ChassisMotion_IsRouteSegmentDone())
      {
        if ((PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_ORIGIN_SIDE) == 0U) ||
            (PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_FAR_SIDE) == 0U))
          EnterFault();
        else
        {
          WorldMap_SetKnownStation(WORLD_STATION_BEAN_AC_PICK);
          if (!StartPickup(BEAN_POSITION_C)) EnterFault();
          else EnterState(BEAN_SEQUENCE_DEMO_PICK_C);
        }
      }
      break;

    case BEAN_SEQUENCE_DEMO_PICK_B:
      if (PickupCompleted()) BeginGripHold(BEAN_POSITION_B);
      break;

    case BEAN_SEQUENCE_DEMO_PICK_C:
      if (PickupCompleted()) BeginGripHold(BEAN_POSITION_C);
      break;

    case BEAN_SEQUENCE_DEMO_PICK_A:
      if (PickupCompleted()) BeginGripHold(BEAN_POSITION_A);
      break;

    case BEAN_SEQUENCE_DEMO_HOLD:
      /* 夹爪在Z最高点闭合保持4秒，专门观察夹持期间是否漏豆。 */
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(BEAN_SEQUENCE_DEMO_RETURN_DESCEND);
      break;

    case BEAN_SEQUENCE_DEMO_RETURN_DESCEND:
    {
      const WorldSlotPose *slot = WorldMap_GetSlot(k_sequence_slots[g_current_position]);
      int32_t release_z;
      if ((slot == 0) || !slot->calibrated)
      {
        EnterFault();
        break;
      }
      /* 闭爪后的爪尖更低，只下降到抓取高度上方5cm再张爪。 */
      release_z = ReleaseZForSlot(slot);
      if (!g_stage_started)
      {
        uint8_t started = StartZMoveTo(release_z);
        if (started == 0U) EnterFault();
        else if (started == 2U)
        {
          StartRelease();
          EnterState(BEAN_SEQUENCE_DEMO_RELEASE);
        }
        else
        {
          g_stage_started = 1U;
          g_deadline = HAL_GetTick() + BEAN_SEQUENCE_Z_MOVE_TIMEOUT_MS;
        }
      }
      else if (ZMoveFinishedAt(release_z))
      {
        WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X), 1U,
                                 release_z, 1U);
        StartRelease();
        EnterState(BEAN_SEQUENCE_DEMO_RELEASE);
      }
      else if (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z) ||
               ((int32_t)(HAL_GetTick() - g_deadline) >= 0)) EnterFault();
      break;
    }

    case BEAN_SEQUENCE_DEMO_RELEASE:
      if ((int32_t)(HAL_GetTick() - g_deadline) >= 0)
        EnterState(BEAN_SEQUENCE_DEMO_RETURN_RAISE);
      break;

    case BEAN_SEQUENCE_DEMO_RETURN_RAISE:
      if (!g_stage_started)
      {
        uint8_t started = StartZMoveTo(0);
        if (started == 0U) EnterFault();
        else if (started == 2U) g_stage_started = 2U;
        else
        {
          g_stage_started = 1U;
          g_deadline = HAL_GetTick() + BEAN_SEQUENCE_Z_MOVE_TIMEOUT_MS;
        }
      }
      if ((g_stage_started == 2U) || ZMoveFinishedAt(0))
      {
        WorldMap_SetAxisPosition(StepperAxis_GetPositionPulses(STEPPER_AXIS_X), 1U,
                                 0, 1U);
        if (g_current_position == BEAN_POSITION_B)
          EnterState(BEAN_SEQUENCE_DEMO_MOVE_TO_AC);
        else if (g_current_position == BEAN_POSITION_C)
        {
          if (!StartPickup(BEAN_POSITION_A)) EnterFault();
          else EnterState(BEAN_SEQUENCE_DEMO_PICK_A);
        }
        else EnterState(BEAN_SEQUENCE_DEMO_COMPLETE);
      }
      else if ((g_stage_started == 1U) &&
               (!StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z) ||
                ((int32_t)(HAL_GetTick() - g_deadline) >= 0))) EnterFault();
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
         (g_state <= BEAN_SEQUENCE_DEMO_RETURN_RAISE);
}

uint8_t BeanSequenceDemo_GetCurrentPosition(void)
{
  return g_current_position;
}
