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
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    MotorControl_SetTargetRpm(i, rpm);
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
  }
  DcMotor_StopAll();
}

void MotorControl_Update(uint8_t enabled)
{
  AppConfig config;
  float average_rpm = 0.0f;
  float dt;
  int32_t delta[APP_MOTOR_COUNT];
  AppConfig_GetSnapshot(&config);
  dt = (float)config.motor_control_period_ms / 1000.0f;

  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    float raw_rpm;
    delta[i] = Encoder_GetDelta(i);
    /* 由周期内脉冲增量换算输出轴 RPM，再做一阶低通滤波。 */
    raw_rpm = ((float)delta[i] * 60000.0f) /
              (config.encoder_counts_per_output_rev * (float)config.motor_control_period_ms);
    g_loops[i].measured_rpm += config.speed_filter_alpha * (raw_rpm - g_loops[i].measured_rpm);
    average_rpm += g_loops[i].measured_rpm;
  }
  /* 平均速度用于四轮同步补偿，减少直行时各轮速度偏差。 */
  average_rpm /= (float)APP_MOTOR_COUNT;

  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    float target = enabled ? g_loops[i].requested_rpm : 0.0f;
    float ramp_rate = (target == 0.0f) ? config.deceleration_rpm_s : config.acceleration_rpm_s;
    float error;
    float output;
    int16_t command;

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
    else if (!enabled)
    {
      g_loops[i].integral = 0.0f;
    }
    output = ClampFloat(output, -(float)config.pwm_max, (float)config.pwm_max);
    command = (int16_t)output;
    DcMotor_SetCommand(i, command);
    AppState_SetMotorTelemetry(i, Encoder_GetCount(i), g_loops[i].ramped_rpm,
                               g_loops[i].measured_rpm, command);
  }
}

void MotorControl_UpdateOpenLoopSingle(uint8_t index, int16_t pwm_command)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);

  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
  {
    int32_t delta = Encoder_GetDelta(i);
    float raw_rpm = ((float)delta * 60000.0f) /
                    (config.encoder_counts_per_output_rev *
                     (float)config.motor_control_period_ms);
    int16_t output = (i == index) ? pwm_command : 0;

    g_loops[i].requested_rpm = 0.0f;
    g_loops[i].ramped_rpm = 0.0f;
    g_loops[i].integral = 0.0f;
    g_loops[i].measured_rpm +=
        config.speed_filter_alpha * (raw_rpm - g_loops[i].measured_rpm);
    if (i == index)
    {
      DcMotor_SetCommand(i, output);
    }
    else
    {
      /* 当前M1调试占用TIM3_CH1/CH2，不能让旧M2逻辑覆盖CH2。 */
      output = 0;
    }
    AppState_SetMotorTelemetry(i, Encoder_GetCount(i), 0.0f,
                               g_loops[i].measured_rpm, output);
  }
}

void MotorControl_UpdatePidSingle(uint8_t index, float target_rpm)
{
  AppConfig config;
  MotorLoop *loop;
  int32_t delta;
  float dt;
  float raw_rpm;
  float error;
  float output;

  if (index >= APP_MOTOR_COUNT)
  {
    return;
  }
  AppConfig_GetSnapshot(&config);
  loop = &g_loops[index];
  dt = (float)config.motor_control_period_ms / 1000.0f;
  loop->requested_rpm = ClampFloat(target_rpm, -config.maximum_rpm, config.maximum_rpm);

  delta = Encoder_GetDelta(index);
  raw_rpm = ((float)delta * 60000.0f) /
            (config.encoder_counts_per_output_rev *
             (float)config.motor_control_period_ms);
  loop->measured_rpm += config.speed_filter_alpha * (raw_rpm - loop->measured_rpm);

  loop->ramped_rpm = MoveTowards(loop->ramped_rpm, loop->requested_rpm,
                                 ((loop->requested_rpm == 0.0f) ?
                                  config.deceleration_rpm_s : config.acceleration_rpm_s) * dt);
  error = loop->ramped_rpm - loop->measured_rpm;
  loop->integral = ClampFloat(loop->integral + error * config.speed_ki[index] * dt,
                              -(float)config.pwm_max, (float)config.pwm_max);
  output = loop->ramped_rpm * config.speed_feedforward[index] +
           error * config.speed_kp[index] + loop->integral;
  if (loop->ramped_rpm != 0.0f)
  {
    output += (output > 0.0f) ? (float)config.pwm_deadband : -(float)config.pwm_deadband;
  }
  output = ClampFloat(output, -(float)config.pwm_max, (float)config.pwm_max);
  DcMotor_SetCommand(index, (int16_t)output);
  AppState_SetMotorTelemetry(index, Encoder_GetCount(index), loop->ramped_rpm,
                             loop->measured_rpm, (int16_t)output);
}
