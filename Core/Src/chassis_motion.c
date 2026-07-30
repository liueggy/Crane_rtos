#include "chassis_motion.h"

#include "app_config.h"
#include "app_state.h"
#include "encoder.h"
#include "motor_control.h"
#include "photo_sensor.h"
#include "main.h"

#define CHASSIS_OPEN_LOOP_PWM 75
#define CHASSIS_DEFAULT_RPM 50
#define CHASSIS_MIN_RPM 20
#define CHASSIS_ALIGN_TIMEOUT_MIN_MS 100U
#define CHASSIS_ALIGN_TIMEOUT_MAX_MS 3000U
#define CHASSIS_MOTOR_MASK_ORIGIN 0x03U
#define CHASSIS_MOTOR_MASK_FAR    0x0CU
#define CHASSIS_ROUTE_ALIGN_TIMEOUT_DEFAULT_MS 1000U
/* 路线定位以光电门为准；里程只做宽松合理性校验，不参与停车。 */
#define CHASSIS_ROUTE_DISTANCE_MIN_PERCENT 45U
#define CHASSIS_ROUTE_DISTANCE_MAX_PERCENT 165U

static volatile uint8_t g_manual_active;
static volatile uint8_t g_running;
static volatile uint8_t g_reverse;
static volatile uint8_t g_closed_loop;
static volatile int16_t g_target_rpm;
static volatile uint16_t g_align_timeout_ms;
static int16_t g_open_loop_command;
static uint8_t g_applied_closed_loop;
static uint8_t g_route_active;
static uint8_t g_route_done;
static uint8_t g_route_failed;
static uint32_t g_route_deadline_tick;
static volatile uint8_t g_side_running_mask;
static volatile uint8_t g_photo_stop_armed;
static volatile uint8_t g_photo_trigger_mask;
static volatile uint8_t g_photo_stop_enabled;
static uint32_t g_first_side_stop_tick;
static uint16_t g_route_expected_distance_mm;
static int32_t g_route_start_count[APP_MOTOR_COUNT];
static uint8_t g_route_target_landmarks;
static volatile uint8_t g_passed_landmarks[2];

static uint32_t ScaleRouteTimeout(uint16_t nominal_speed_rpm,
                                  uint16_t nominal_timeout_ms)
{
  uint32_t scaled = ((uint32_t)nominal_timeout_ms * nominal_speed_rpm +
                     (uint32_t)g_target_rpm - 1U) /
                    (uint32_t)g_target_rpm;
  return (scaled < 100U) ? 100U : scaled;
}

typedef enum
{
  ROUTE_DISTANCE_TOO_EARLY = 0,
  ROUTE_DISTANCE_ACCEPTABLE,
  ROUTE_DISTANCE_TOO_LATE
} RouteDistanceStatus;

static uint32_t CountDifference(int32_t current, int32_t start)
{
  int64_t difference = (int64_t)current - (int64_t)start;
  return (uint32_t)((difference < 0) ? -difference : difference);
}

static RouteDistanceStatus ChassisMotion_GetSideDistanceStatus(uint8_t side)
{
  AppConfig config;
  uint8_t first_motor = (side == CHASSIS_SIDE_ORIGIN) ? 0U : 2U;
  uint32_t count_a = CountDifference(Encoder_GetCount(first_motor),
                                     g_route_start_count[first_motor]);
  uint32_t count_b = CountDifference(Encoder_GetCount(first_motor + 1U),
                                     g_route_start_count[first_motor + 1U]);
  uint32_t average_count = (count_a + count_b) / 2U;
  uint32_t expected_count;
  uint32_t minimum_count;
  uint32_t maximum_count;

  if (!g_route_active || (g_route_expected_distance_mm == 0U))
    return ROUTE_DISTANCE_ACCEPTABLE;
  AppConfig_GetSnapshot(&config);
  expected_count = (uint32_t)(((float)g_route_expected_distance_mm *
                               config.encoder_counts_per_output_rev /
                               config.wheel_circumference_mm) + 0.5f);
  minimum_count = expected_count * CHASSIS_ROUTE_DISTANCE_MIN_PERCENT / 100U;
  maximum_count = expected_count * CHASSIS_ROUTE_DISTANCE_MAX_PERCENT / 100U;
  if (average_count < minimum_count) return ROUTE_DISTANCE_TOO_EARLY;
  if (average_count > maximum_count) return ROUTE_DISTANCE_TOO_LATE;
  return ROUTE_DISTANCE_ACCEPTABLE;
}

static uint8_t ChassisPhotoMask(void)
{
  uint8_t mask = 0U;
  if (PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_ORIGIN_SIDE)) mask |= 1U;
  if (PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_FAR_SIDE)) mask |= 2U;
  return mask;
}

static void ChassisMotion_ProcessPhotoStop(void)
{
  uint8_t blocked = ChassisPhotoMask();

  if (!g_running || !g_photo_stop_enabled) return;
  for (uint8_t side = CHASSIS_SIDE_ORIGIN; side <= CHASSIS_SIDE_FAR; side <<= 1U)
  {
    if ((g_side_running_mask & side) == 0U) continue;
    /* 本侧已经锁存当前组后继续行驶，但在另一侧确认前不允许重复计数。 */
    if (g_photo_trigger_mask & side) continue;
    if ((g_photo_stop_armed & side) == 0U)
    {
      /* 启动时若已遮挡，先等本侧离开；之后的下一次遮挡才停车。 */
      if ((blocked & side) == 0U)
      {
        g_photo_stop_armed |= side;
        g_photo_trigger_mask &= (uint8_t)~side;
      }
    }
    else if (blocked & side)
    {
      uint8_t side_index = (side == CHASSIS_SIDE_ORIGIN) ? 0U : 1U;
      if ((uint8_t)(g_passed_landmarks[side_index] + 1U) <
          g_route_target_landmarks)
      {
        /* 单侧只锁存当前组事件；两侧均确认前四轮保持运行。 */
        if (g_photo_trigger_mask == 0U) g_first_side_stop_tick = HAL_GetTick();
        ++g_passed_landmarks[side_index];
        g_photo_stop_armed &= (uint8_t)~side;
        g_photo_trigger_mask |= side;
        continue;
      }
      RouteDistanceStatus distance_status =
          ChassisMotion_GetSideDistanceStatus(side);
      /* 光电门决定到站；编码器仅确认本次行程没有明显异常。 */
      if (distance_status == ROUTE_DISTANCE_TOO_EARLY)
      {
        /* 停车后滑出、回弹或挡片边沿抖动仍属于当前地标，不能推进路线。 */
        continue;
      }
      if (distance_status == ROUTE_DISTANCE_TOO_LATE)
      {
        g_route_active = 0U;
        g_route_failed = 1U;
        g_running = 0U;
        g_side_running_mask = 0U;
        MotorControl_Reset();
        AppState_SetRunEnabled(0U);
        return;
      }
      if (g_photo_trigger_mask == 0U) g_first_side_stop_tick = HAL_GetTick();
      g_photo_stop_armed &= (uint8_t)~side;
      g_photo_trigger_mask |= side;
      ++g_passed_landmarks[side_index];
    }
  }
  if ((g_photo_trigger_mask == CHASSIS_SIDE_ALL) &&
      (g_passed_landmarks[0] < g_route_target_landmarks) &&
      (g_passed_landmarks[1] < g_route_target_landmarks))
  {
    /* 中途地标只完成双侧计数并重新布防，不复位速度环、不撤销PWM，
     * 保持四轮连续通过，避免每组挡板都从0重新加速造成顿挫。 */
    g_photo_trigger_mask = 0U;
    g_first_side_stop_tick = 0U;
  }
  else if ((g_photo_trigger_mask == CHASSIS_SIDE_ALL) &&
           (g_passed_landmarks[0] >= g_route_target_landmarks) &&
           (g_passed_landmarks[1] >= g_route_target_landmarks))
  {
    /* 目标组两侧都确认后再让四轮同时停车，取消单边停车。 */
    g_side_running_mask = 0U;
  }
  g_running = (g_side_running_mask != 0U) ? 1U : 0U;
  AppState_SetRunEnabled(g_running);
}

void ChassisMotion_Init(void)
{
  g_manual_active = 0U;
  g_running = 0U;
  g_reverse = 0U;
  g_closed_loop = 1U;
  g_target_rpm = CHASSIS_DEFAULT_RPM;
  g_align_timeout_ms = CHASSIS_ROUTE_ALIGN_TIMEOUT_DEFAULT_MS;
  g_open_loop_command = 0;
  g_applied_closed_loop = 1U;
  g_route_active = 0U;
  g_route_done = 0U;
  g_route_failed = 0U;
  g_route_deadline_tick = 0U;
  g_side_running_mask = 0U;
  g_photo_trigger_mask = ChassisPhotoMask();
  g_photo_stop_armed = 0U;
  g_photo_stop_enabled = 1U;
  g_first_side_stop_tick = 0U;
  g_route_expected_distance_mm = 0U;
  g_route_target_landmarks = 1U;
  g_passed_landmarks[0] = 0U;
  g_passed_landmarks[1] = 0U;
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i) g_route_start_count[i] = 0;
}

void ChassisMotion_ToggleRunning(void)
{
  ChassisMotion_SetRunning(g_running ? 0U : 1U);
}

void ChassisMotion_SetRunning(uint8_t running)
{
  if (running)
  {
    uint8_t blocked = ChassisPhotoMask();
    g_side_running_mask = CHASSIS_SIDE_ALL;
    /* 当前无遮挡侧立即布防；当前遮挡侧先离站，再等待下一次遮挡。 */
    g_photo_stop_armed = (uint8_t)(CHASSIS_SIDE_ALL & (uint8_t)~blocked);
    g_photo_trigger_mask = 0U;
  }
  else
  {
    g_side_running_mask = 0U;
    g_photo_stop_enabled = 1U;
  }
  g_running = running ? 1U : 0U;
  g_manual_active = 1U;
  AppState_SetRunEnabled(g_running);
}

void ChassisMotion_StopManual(void)
{
  ChassisMotion_SetRunning(0U);
}

uint8_t ChassisMotion_IsRunning(void)
{
  return g_running;
}

uint8_t ChassisMotion_IsPhotoStopArmed(void)
{
  return g_photo_stop_armed;
}

uint8_t ChassisMotion_GetPhotoTriggerMask(void)
{
  return g_photo_trigger_mask;
}

uint8_t ChassisMotion_GetRunningSideMask(void)
{
  return g_side_running_mask;
}

void ChassisMotion_SetPhotoStopEnabled(uint8_t enabled)
{
  if (!g_running) g_photo_stop_enabled = enabled ? 1U : 0U;
}

uint8_t ChassisMotion_SetDirection(uint8_t reverse)
{
  AppState state;
  AppState_GetSnapshot(&state);
  if (g_running) return 0U;
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    if (state.pwm_command[i] != 0) return 0U;
  }
  g_reverse = reverse ? 1U : 0U;
  g_manual_active = 1U;
  return 1U;
}

uint8_t ChassisMotion_IsDirectionReverse(void)
{
  return g_reverse;
}

uint8_t ChassisMotion_ToggleClosedLoop(void)
{
  return ChassisMotion_SetClosedLoop(g_closed_loop ? 0U : 1U);
}

uint8_t ChassisMotion_SetClosedLoop(uint8_t enabled)
{
  AppState state;
  AppState_GetSnapshot(&state);
  if (g_running) return 0U;
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    if (state.pwm_command[i] != 0) return 0U;
  }
  g_closed_loop = enabled ? 1U : 0U;
  g_manual_active = 1U;
  return 1U;
}

uint8_t ChassisMotion_IsClosedLoop(void)
{
  return g_closed_loop;
}

void ChassisMotion_AdjustTargetRpm(int16_t delta_rpm)
{
  ChassisMotion_SetTargetRpm((int16_t)(g_target_rpm + delta_rpm));
}

void ChassisMotion_SetTargetRpm(int16_t target_rpm)
{
  AppConfig config;
  int32_t target = target_rpm;
  AppConfig_GetSnapshot(&config);
  if (target < CHASSIS_MIN_RPM) target = CHASSIS_MIN_RPM;
  if (target > (int32_t)config.maximum_rpm) target = (int32_t)config.maximum_rpm;
  g_target_rpm = (int16_t)target;
}

int16_t ChassisMotion_GetTargetRpm(void)
{
  return g_target_rpm;
}

void ChassisMotion_AdjustAlignTimeout(int16_t delta_ms)
{
  int32_t timeout;
  if (g_running) return;
  timeout = (int32_t)g_align_timeout_ms + delta_ms;
  if (timeout < CHASSIS_ALIGN_TIMEOUT_MIN_MS)
    timeout = CHASSIS_ALIGN_TIMEOUT_MIN_MS;
  if (timeout > CHASSIS_ALIGN_TIMEOUT_MAX_MS)
    timeout = CHASSIS_ALIGN_TIMEOUT_MAX_MS;
  g_align_timeout_ms = (uint16_t)timeout;
}

uint16_t ChassisMotion_GetAlignTimeoutMs(void)
{
  return g_align_timeout_ms;
}

void ChassisMotion_Stop(void)
{
  g_manual_active = 0U;
  g_running = 0U;
  g_side_running_mask = 0U;
  g_open_loop_command = 0;
  g_route_active = 0U;
  g_route_done = 0U;
  g_route_failed = 0U;
  g_route_expected_distance_mm = 0U;
  g_route_target_landmarks = 1U;
  g_passed_landmarks[0] = 0U;
  g_passed_landmarks[1] = 0U;
  g_photo_stop_enabled = 1U;
  MotorControl_Reset();
  AppState_SetRunEnabled(0U);
}

uint8_t ChassisMotion_StartRouteSegment(int16_t distance_mm, int16_t turn_deg,
                                        uint16_t speed_rpm, uint16_t timeout_ms)
{
  if (turn_deg != 0) return 0U;
  return ChassisMotion_StartRouteThroughLandmarks(distance_mm, 1U, speed_rpm,
                                                   timeout_ms);
}

uint8_t ChassisMotion_StartRouteThroughLandmarks(int16_t distance_mm,
                                                 uint8_t landmark_count,
                                                 uint16_t speed_rpm,
                                                 uint16_t timeout_ms)
{
  uint8_t blocked;
  if ((distance_mm == 0) || (landmark_count == 0U) || (speed_rpm == 0U) ||
      (timeout_ms < 100U)) return 0U;

  blocked = ChassisPhotoMask();
  g_photo_stop_enabled = 1U;
  g_manual_active = 0U;
  g_reverse = (distance_mm < 0) ? 1U : 0U;
  g_side_running_mask = CHASSIS_SIDE_ALL;
  g_photo_stop_armed = (uint8_t)(CHASSIS_SIDE_ALL & (uint8_t)~blocked);
  g_photo_trigger_mask = 0U;
  g_running = 1U;
  g_route_active = 1U;
  g_route_done = 0U;
  g_route_failed = 0U;
  g_route_deadline_tick = HAL_GetTick() +
                          ScaleRouteTimeout(speed_rpm, timeout_ms);
  g_first_side_stop_tick = 0U;
  g_route_expected_distance_mm = (uint16_t)((distance_mm < 0) ?
                                            -distance_mm : distance_mm);
  g_route_target_landmarks = landmark_count;
  g_passed_landmarks[0] = 0U;
  g_passed_landmarks[1] = 0U;
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
    g_route_start_count[i] = Encoder_GetCount(i);
  MotorControl_Reset();
  AppState_SetRunEnabled(1U);
  return 1U;
}

uint8_t ChassisMotion_StartPhotoLandmarkRoute(uint8_t reverse,
                                              uint8_t landmark_count,
                                              uint16_t speed_rpm,
                                              uint16_t timeout_ms)
{
  uint8_t blocked;
  if ((landmark_count == 0U) || (speed_rpm == 0U) ||
      (timeout_ms < 100U)) return 0U;

  blocked = ChassisPhotoMask();
  g_photo_stop_enabled = 1U;
  g_manual_active = 0U;
  g_reverse = reverse ? 1U : 0U;
  g_side_running_mask = CHASSIS_SIDE_ALL;
  g_photo_stop_armed = (uint8_t)(CHASSIS_SIDE_ALL & (uint8_t)~blocked);
  g_photo_trigger_mask = 0U;
  g_running = 1U;
  g_route_active = 1U;
  g_route_done = 0U;
  g_route_failed = 0U;
  g_route_deadline_tick = HAL_GetTick() +
                          ScaleRouteTimeout(speed_rpm, timeout_ms);
  g_first_side_stop_tick = 0U;
  g_route_expected_distance_mm = 0U;
  g_route_target_landmarks = landmark_count;
  g_passed_landmarks[0] = 0U;
  g_passed_landmarks[1] = 0U;
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
    g_route_start_count[i] = Encoder_GetCount(i);
  MotorControl_Reset();
  AppState_SetRunEnabled(1U);
  return 1U;
}

uint8_t ChassisMotion_GetPassedLandmarkCount(uint8_t side)
{
  if (side == CHASSIS_SIDE_ORIGIN) return g_passed_landmarks[0];
  if (side == CHASSIS_SIDE_FAR) return g_passed_landmarks[1];
  return 0U;
}

uint8_t ChassisMotion_IsRouteSegmentDone(void)
{
  return g_route_done;
}

uint8_t ChassisMotion_DidRouteSegmentFail(void)
{
  return g_route_failed;
}

void ChassisMotion_TaskStep(void)
{
  AppState state;
  ChassisMotion_ProcessPhotoStop();
  AppState_GetSnapshot(&state);
  if (state.estop_active)
  {
    ChassisMotion_Stop();
    return;
  }

  if (g_route_active)
  {
    uint8_t motor_mask = 0U;
    float target = g_reverse ? -(float)g_target_rpm : (float)g_target_rpm;
    if ((int32_t)(HAL_GetTick() - g_route_deadline_tick) >= 0)
    {
      g_route_active = 0U;
      g_route_failed = 1U;
      g_running = 0U;
      g_side_running_mask = 0U;
      MotorControl_Reset();
      AppState_SetRunEnabled(0U);
      return;
    }
    if ((g_photo_trigger_mask != 0U) &&
        (g_photo_trigger_mask != CHASSIS_SIDE_ALL) &&
        ((uint32_t)(HAL_GetTick() - g_first_side_stop_tick) >
         g_align_timeout_ms))
    {
      /* 一侧已到站而另一侧长期未到，判定为挡板漏检或车架明显歪斜。 */
      g_route_active = 0U;
      g_route_failed = 1U;
      g_running = 0U;
      g_side_running_mask = 0U;
      MotorControl_Reset();
      AppState_SetRunEnabled(0U);
      return;
    }
    if (g_side_running_mask == 0U)
    {
      g_route_active = 0U;
      g_route_done = 1U;
      g_running = 0U;
      MotorControl_Reset();
      AppState_SetRunEnabled(0U);
      return;
    }
    MotorControl_SetAllTargetRpmCompensated(target);
    if (g_side_running_mask & CHASSIS_SIDE_ORIGIN)
      motor_mask |= CHASSIS_MOTOR_MASK_ORIGIN;
    if (g_side_running_mask & CHASSIS_SIDE_FAR)
      motor_mask |= CHASSIS_MOTOR_MASK_FAR;
    MotorControl_UpdateMasked(motor_mask);
    return;
  }

  if (!g_manual_active)
  {
    MotorControl_Update(0U);
    return;
  }

  if (g_closed_loop)
  {
    float target = g_reverse ? -(float)g_target_rpm : (float)g_target_rpm;
    uint8_t motor_mask = 0U;
    if (!g_applied_closed_loop)
    {
      MotorControl_Reset();
      g_applied_closed_loop = 1U;
    }
    MotorControl_SetAllTargetRpmCompensated(target);
    if (g_side_running_mask & CHASSIS_SIDE_ORIGIN)
      motor_mask |= CHASSIS_MOTOR_MASK_ORIGIN;
    if (g_side_running_mask & CHASSIS_SIDE_FAR)
      motor_mask |= CHASSIS_MOTOR_MASK_FAR;
    MotorControl_UpdateMasked(motor_mask);
    return;
  }

  if (g_applied_closed_loop)
  {
    MotorControl_Reset();
    g_open_loop_command = 0;
    g_applied_closed_loop = 0U;
  }
  if (!g_running)
    g_open_loop_command = 0;
  else
    g_open_loop_command = g_reverse ? -CHASSIS_OPEN_LOOP_PWM : CHASSIS_OPEN_LOOP_PWM;

  int16_t commands[APP_MOTOR_COUNT];
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    uint8_t side_running = (i < 2U) ?
        (g_side_running_mask & CHASSIS_SIDE_ORIGIN) :
        (g_side_running_mask & CHASSIS_SIDE_FAR);
    commands[i] = side_running ? g_open_loop_command : 0;
  }
  MotorControl_UpdateOpenLoop(commands);
}
