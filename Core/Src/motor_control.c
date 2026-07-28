#include "motor_control.h"

#include "app_config.h"
#include "app_state.h"
#include "dc_motor.h"
#include "encoder.h"

typedef struct
{
  float requested_rpm;
  float ramped_rpm;
  float measured_rpm;
  float integral;
  int32_t encoder_delta_accumulator;
  uint16_t measurement_elapsed_ms;
} MotorLoop;

/* 每个轮子独立保存目标、斜坡目标、测速值和 PI 积分项。 */
static MotorLoop g_loops[APP_MOTOR_COUNT];

static float ClampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static float MoveTowards(float current, float target, float maximum_step)
{
  if (target > current + maximum_step) return current + maximum_step;
  if (target < current - maximum_step) return current - maximum_step;
  return target;
}

static float UpdateMeasuredRpm(uint8_t index, const AppConfig *config)
{
  MotorLoop *loop = &g_loops[index];
  loop->encoder_delta_accumulator += Encoder_GetDelta(index);
  loop->measurement_elapsed_ms =
      (uint16_t)(loop->measurement_elapsed_ms + config->motor_control_period_ms);

  if (loop->measurement_elapsed_ms >= config->speed_measurement_period_ms)
  {
    float raw_rpm = ((float)loop->encoder_delta_accumulator * 60000.0f) /
                    (config->encoder_counts_per_output_rev *
                     (float)loop->measurement_elapsed_ms);
    loop->measured_rpm +=
        config->speed_filter_alpha * (raw_rpm - loop->measured_rpm);
    loop->encoder_delta_accumulator = 0;
    loop->measurement_elapsed_ms = 0U;
  }
  return loop->measured_rpm;
}

void MotorControl_Init(void)
{
  MotorControl_Reset();
}

void MotorControl_SetTargetRpm(uint8_t index, float rpm)
{
  AppConfig config;
  if (index >= APP_MOTOR_COUNT) return;
  /* 一个控制周期使用同一份配置，避免热更新造成计算不一致。 */
  AppConfig_GetSnapshot(&config);
  g_loops[index].requested_rpm = ClampFloat(rpm, -config.maximum_rpm, config.maximum_rpm);
}

void MotorControl_SetAllTargetRpm(float rpm)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);
  rpm = ClampFloat(rpm, -config.maximum_rpm, config.maximum_rpm);

  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    g_loops[i].requested_rpm = rpm;
  }
}

void MotorControl_SetAllTargetRpmCompensated(float rpm)
{
  AppConfig config;
  float direction;
  AppConfig_GetSnapshot(&config);
  rpm = ClampFloat(rpm, -config.maximum_rpm, config.maximum_rpm);
  direction = (rpm > 0.0f) ? 1.0f : ((rpm < 0.0f) ? -1.0f : 0.0f);

  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    float target = rpm + direction * config.motor_target_trim_rpm[i];
    g_loops[i].requested_rpm = ClampFloat(target, -config.maximum_rpm,
                                           config.maximum_rpm);
  }
}

void MotorControl_Reset(void)
{
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    g_loops[i].requested_rpm = 0.0f;
    g_loops[i].ramped_rpm = 0.0f;
    g_loops[i].measured_rpm = 0.0f;
    g_loops[i].integral = 0.0f;
    g_loops[i].encoder_delta_accumulator = 0;
    g_loops[i].measurement_elapsed_ms = 0U;
  }
  DcMotor_StopAll();
}

void MotorControl_Update(uint8_t enabled)
{
  MotorControl_UpdateMasked(enabled ? 0x0FU : 0U);
}

void MotorControl_UpdateMasked(uint8_t enabled_mask)
{
  AppConfig config;
  float average_rpm = 0.0f;
  float dt;
  uint8_t enabled_count = 0U;
  AppConfig_GetSnapshot(&config);
  dt = (float)config.motor_control_period_ms / 1000.0f;

  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    g_loops[i].measured_rpm = UpdateMeasuredRpm(i, &config);
    if (enabled_mask & (uint8_t)(1U << i))
    {
      average_rpm += g_loops[i].measured_rpm;
      ++enabled_count;
    }
  }

  /* 手动停止必须立即撤销桥臂输出，不能继续用残余误差主动制动。 */
  if (enabled_mask == 0U)
  {
    for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
    {
      g_loops[i].requested_rpm = 0.0f;
      g_loops[i].ramped_rpm = 0.0f;
      g_loops[i].integral = 0.0f;
      DcMotor_SetCommand(i, 0);
      AppState_SetMotorTelemetry(i, Encoder_GetCount(i), 0.0f,
                                 g_loops[i].measured_rpm, 0);
    }
    return;
  }
  /* 平均速度用于四轮同步补偿，减少直行时各轮速度偏差。 */
  average_rpm /= (float)enabled_count;

  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    uint8_t enabled = (enabled_mask & (uint8_t)(1U << i)) ? 1U : 0U;
    float target = enabled ? g_loops[i].requested_rpm : 0.0f;
    float ramp_rate = (target == 0.0f) ? config.deceleration_rpm_s : config.acceleration_rpm_s;
    float error;
    float output;
    int16_t command;

    if (!enabled)
    {
      g_loops[i].requested_rpm = 0.0f;
      g_loops[i].ramped_rpm = 0.0f;
      g_loops[i].integral = 0.0f;
      DcMotor_SetCommand(i, 0);
      AppState_SetMotorTelemetry(i, Encoder_GetCount(i), 0.0f,
                                 g_loops[i].measured_rpm, 0);
      continue;
    }
    /* 先限制目标变化速度，再进入 PI，避免轮子突然启动。 */
    g_loops[i].ramped_rpm = MoveTowards(g_loops[i].ramped_rpm, target, ramp_rate * dt);
    error = g_loops[i].ramped_rpm - g_loops[i].measured_rpm;
    g_loops[i].integral = ClampFloat(g_loops[i].integral + error * config.speed_ki[i] * dt,
                                     -(float)config.pwm_max, (float)config.pwm_max);
    /* 前馈负责主要驱动力，PI 负责消除负载和电机差异造成的误差。 */
    output = g_loops[i].ramped_rpm * config.speed_feedforward[i] +
             error * config.speed_kp[i] + g_loops[i].integral;
    if (g_loops[i].ramped_rpm != 0.0f)
    {
      output += (average_rpm - g_loops[i].measured_rpm) * config.speed_sync_kp;
      output += (output > 0.0f) ? (float)config.pwm_deadband : -(float)config.pwm_deadband;
    }
    /* 速度环只减小同向驱动力，不允许超调时跨零主动反转。 */
    if (g_loops[i].ramped_rpm > 0.0f)
      output = ClampFloat(output, 0.0f, (float)config.pwm_max);
    else if (g_loops[i].ramped_rpm < 0.0f)
      output = ClampFloat(output, -(float)config.pwm_max, 0.0f);
    else
      output = 0.0f;
    command = (int16_t)output;
    DcMotor_SetCommand(i, command);
    AppState_SetMotorTelemetry(i, Encoder_GetCount(i), g_loops[i].ramped_rpm,
                               g_loops[i].measured_rpm, command);
  }
}

void MotorControl_UpdateOpenLoop(const int16_t pwm_command[APP_MOTOR_COUNT])
{
  AppConfig config;
  if (pwm_command == NULL) return;
  AppConfig_GetSnapshot(&config);

  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    g_loops[i].requested_rpm = 0.0f;
    g_loops[i].ramped_rpm = 0.0f;
    g_loops[i].integral = 0.0f;
    g_loops[i].measured_rpm = UpdateMeasuredRpm(i, &config);
    DcMotor_SetCommand(i, pwm_command[i]);
    AppState_SetMotorTelemetry(i, Encoder_GetCount(i), 0.0f,
                               g_loops[i].measured_rpm, pwm_command[i]);
  }
}
