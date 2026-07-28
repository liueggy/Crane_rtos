#include "odometry_calibration.h"

#include "app_config.h"
#include "chassis_motion.h"
#include "encoder.h"
#include "robot_controller.h"
#include "safety_manager.h"

#define ODOMETRY_DEFAULT_DISTANCE_MM 500U
#define ODOMETRY_DEFAULT_SPEED_RPM    40U
#define ODOMETRY_MAX_DISTANCE_MM    9999U
#define ODOMETRY_MIN_SPEED_RPM        20U
#define ODOMETRY_TIMEOUT_MARGIN_MS   3000U
#define ODOMETRY_TIMEOUT_FACTOR         2U

static OdometryCalibrationState g_state;
static OdometryCalibrationField g_selected_field;
static uint16_t g_target_distance_mm;
static uint16_t g_target_speed_rpm;
static uint8_t g_reverse;
static int32_t g_start_count[APP_MOTOR_COUNT];
static uint32_t g_wheel_count[APP_MOTOR_COUNT];
static uint32_t g_target_average_count;
static uint32_t g_average_count;
static uint32_t g_spread_count;
static uint32_t g_deadline;

static uint32_t AbsoluteDelta(int32_t current, int32_t start)
{
  int64_t delta = (int64_t)current - start;
  return (uint32_t)((delta < 0) ? -delta : delta);
}

static uint32_t CountsToMmX10(uint32_t counts)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);
  return (uint32_t)(((float)counts * config.wheel_circumference_mm * 10.0f /
                     config.encoder_counts_per_output_rev) + 0.5f);
}

static void UpdateMeasurement(void)
{
  uint32_t minimum = UINT32_MAX;
  uint32_t maximum = 0U;
  uint32_t sum = 0U;

  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    uint32_t delta = AbsoluteDelta(Encoder_GetCount(i), g_start_count[i]);
    g_wheel_count[i] = delta;
    if (delta < minimum) minimum = delta;
    if (delta > maximum) maximum = delta;
    sum += delta;
  }
  g_average_count = sum / APP_MOTOR_COUNT;
  g_spread_count = maximum - minimum;
}

void OdometryCalibration_Init(void)
{
  g_state = ODOMETRY_CALIBRATION_IDLE;
  g_selected_field = ODOMETRY_FIELD_DISTANCE;
  g_target_distance_mm = ODOMETRY_DEFAULT_DISTANCE_MM;
  g_target_speed_rpm = ODOMETRY_DEFAULT_SPEED_RPM;
  g_reverse = 0U;
  g_target_average_count = 0U;
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i) g_wheel_count[i] = 0U;
  g_average_count = 0U;
  g_spread_count = 0U;
  g_deadline = 0U;
}

void OdometryCalibration_SelectField(OdometryCalibrationField field)
{
  if (OdometryCalibration_IsRunning()) return;
  g_selected_field = field;
  if (field == ODOMETRY_FIELD_DISTANCE) g_target_distance_mm = 0U;
  else g_target_speed_rpm = 0U;
}

void OdometryCalibration_AppendDigit(uint8_t digit)
{
  AppConfig config;
  uint32_t value;

  if (OdometryCalibration_IsRunning() || (digit > 9U)) return;
  AppConfig_GetSnapshot(&config);
  if (g_selected_field == ODOMETRY_FIELD_DISTANCE)
  {
    value = (uint32_t)g_target_distance_mm * 10U + digit;
    if (value <= ODOMETRY_MAX_DISTANCE_MM)
      g_target_distance_mm = (uint16_t)value;
  }
  else
  {
    value = (uint32_t)g_target_speed_rpm * 10U + digit;
    if (value <= (uint32_t)config.maximum_rpm)
      g_target_speed_rpm = (uint16_t)value;
  }
}

void OdometryCalibration_SetDirection(uint8_t reverse)
{
  if (!OdometryCalibration_IsRunning()) g_reverse = reverse ? 1U : 0U;
}

void OdometryCalibration_ToggleRunning(void)
{
  AppConfig config;
  RobotState robot_state;
  uint32_t expected_ms;

  if (OdometryCalibration_IsRunning())
  {
    OdometryCalibration_Abort();
    return;
  }
  robot_state = RobotController_GetState();
  if ((g_target_distance_mm == 0U) ||
      (g_target_speed_rpm < ODOMETRY_MIN_SPEED_RPM) ||
      ((robot_state != ROBOT_STATE_IDLE) &&
       (robot_state != ROBOT_STATE_CALIBRATION_REQUIRED) &&
       (robot_state != ROBOT_STATE_FINISHED)) ||
      (SafetyManager_GetFlags() & (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)))
  {
    g_state = ODOMETRY_CALIBRATION_FAULT;
    return;
  }

  AppConfig_GetSnapshot(&config);
  g_target_average_count = (uint32_t)(
      ((float)g_target_distance_mm * config.encoder_counts_per_output_rev /
       config.wheel_circumference_mm) + 0.5f);
  if (g_target_average_count == 0U)
  {
    g_state = ODOMETRY_CALIBRATION_FAULT;
    return;
  }

  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    g_start_count[i] = Encoder_GetCount(i);
    g_wheel_count[i] = 0U;
  }
  g_average_count = 0U;
  g_spread_count = 0U;

  ChassisMotion_StopManual();
  if (!ChassisMotion_SetClosedLoop(1U) ||
      !ChassisMotion_SetDirection(g_reverse))
  {
    g_state = ODOMETRY_CALIBRATION_FAULT;
    return;
  }
  ChassisMotion_SetTargetRpm((int16_t)g_target_speed_rpm);
  /* 标定只按编码器距离停车，运行期间明确旁路PD8/PB12挡片。 */
  ChassisMotion_SetPhotoStopEnabled(0U);
  ChassisMotion_SetRunning(1U);
  if (!ChassisMotion_IsRunning())
  {
    g_state = ODOMETRY_CALIBRATION_FAULT;
    return;
  }

  expected_ms = (uint32_t)(((float)g_target_distance_mm * 60000.0f /
                            ((float)g_target_speed_rpm *
                             config.wheel_circumference_mm)) + 0.5f);
  g_deadline = HAL_GetTick() + expected_ms * ODOMETRY_TIMEOUT_FACTOR +
               ODOMETRY_TIMEOUT_MARGIN_MS;
  g_state = ODOMETRY_CALIBRATION_RUNNING;
}

void OdometryCalibration_Abort(void)
{
  if (OdometryCalibration_IsRunning())
  {
    ChassisMotion_StopManual();
    UpdateMeasurement();
    g_state = ODOMETRY_CALIBRATION_STOPPED;
  }
}

void OdometryCalibration_Process(void)
{
  if (!OdometryCalibration_IsRunning()) return;

  UpdateMeasurement();
  if (SafetyManager_GetFlags() & (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT))
  {
    ChassisMotion_StopManual();
    g_state = ODOMETRY_CALIBRATION_FAULT;
  }
  else if (g_average_count >= g_target_average_count)
  {
    ChassisMotion_StopManual();
    UpdateMeasurement();
    g_state = ODOMETRY_CALIBRATION_COMPLETE;
  }
  else if (!ChassisMotion_IsRunning() ||
           ((int32_t)(HAL_GetTick() - g_deadline) >= 0))
  {
    ChassisMotion_StopManual();
    g_state = ODOMETRY_CALIBRATION_FAULT;
  }
}

OdometryCalibrationState OdometryCalibration_GetState(void)
{
  return g_state;
}

OdometryCalibrationField OdometryCalibration_GetSelectedField(void)
{
  return g_selected_field;
}

uint16_t OdometryCalibration_GetTargetDistanceMm(void)
{
  return g_target_distance_mm;
}

uint16_t OdometryCalibration_GetTargetSpeedRpm(void)
{
  return g_target_speed_rpm;
}

uint8_t OdometryCalibration_GetDirectionReverse(void)
{
  return g_reverse;
}

uint32_t OdometryCalibration_GetMeasuredDistanceMmX10(void)
{
  return CountsToMmX10(g_average_count);
}

uint32_t OdometryCalibration_GetWheelSpreadMmX10(void)
{
  return CountsToMmX10(g_spread_count);
}

uint32_t OdometryCalibration_GetWheelDistanceMmX10(uint8_t index)
{
  return (index < APP_MOTOR_COUNT) ? CountsToMmX10(g_wheel_count[index]) : 0U;
}

uint8_t OdometryCalibration_IsRunning(void)
{
  return g_state == ODOMETRY_CALIBRATION_RUNNING;
}
