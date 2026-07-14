#include "ui_manager.h"

#include "app_config.h"
#include "app_state.h"
#include "font.h"
#include "oled.h"
#include "cmsis_os2.h"
#include <stdio.h>

#define OLED_I2C_ADDRESS 0x78U

/* 当前页面只保存页面编号，具体内容由 Render 根据状态快照绘制。 */
static UiPage g_page = UI_PAGE_OVERVIEW;

static void UiDelay(uint32_t delay_ms)
{
  if (osKernelGetState() == osKernelRunning) osDelay(delay_ms);
  else HAL_Delay(delay_ms);
}

static int RoundedInt(float value)
{
  return (int)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static void DrawLine(uint8_t y, const char *text)
{
  /* 中文界面使用 16x16 字库；ASCII 字符由字库的回退字体绘制。 */
  OLED_PrintString(0, y, (char *)text, &font16x16, OLED_COLOR_NORMAL);
}

static void DrawAsciiLine(uint8_t y, const char *text)
{
  OLED_PrintASCIIString(0, y, text, &afont12x6, OLED_COLOR_NORMAL);
}

static void DrawHeader(const char *title)
{
  char page[8];
  OLED_PrintString(0, 0, (char *)title, &font16x16, OLED_COLOR_NORMAL);
  (void)snprintf(page, sizeof(page), "%u/%u", (unsigned)(g_page + 1U),
                 (unsigned)UI_PAGE_COUNT);
  OLED_PrintASCIIString(96, 0, page, &afont12x6, OLED_COLOR_NORMAL);
}

static void DrawBootFrame(uint8_t progress)
{
  char line[22];
  OLED_NewFrame();
  OLED_PrintString(24, 8, "起重机", &font16x16, OLED_COLOR_NORMAL);
  OLED_PrintString(28, 30, "系统启动", &font16x16, OLED_COLOR_NORMAL);
  OLED_DrawRectangle(14, 48, 100, 8, OLED_COLOR_NORMAL);
  OLED_DrawFilledRectangle(16, 50, (uint8_t)(progress * 96U / 100U), 4, OLED_COLOR_NORMAL);
  (void)snprintf(line, sizeof(line), "%3u%%", (unsigned)progress);
  OLED_PrintASCIIString(52, 58, line, &afont12x6, OLED_COLOR_NORMAL);
  OLED_ShowFrame();
}

static void PlayBootAnimation(void)
{
  static const uint8_t progress[] = {0U, 25U, 50U, 75U, 100U};
  const uint8_t progress_count = (uint8_t)(sizeof(progress) / sizeof(progress[0]));
  for (uint8_t i = 0U; i < progress_count; ++i)
  {
    DrawBootFrame(progress[i]);
    UiDelay(90U);
  }
}

void UiManager_Init(I2C_HandleTypeDef *i2c)
{
  OLED_Init(i2c, OLED_I2C_ADDRESS);
  PlayBootAnimation();
  UiManager_SetPage(UI_PAGE_OVERVIEW);
}

void UiManager_NextPage(void)
{
  UiManager_SetPage((UiPage)((g_page + 1U) % UI_PAGE_COUNT));
}

void UiManager_SetPage(UiPage page)
{
  if (page >= UI_PAGE_COUNT) page = UI_PAGE_OVERVIEW;
  g_page = page;
  AppState_SetUiPage((uint8_t)page);
}

UiPage UiManager_GetPage(void)
{
  return g_page;
}

void UiManager_Render(void)
{
  AppConfig config;
  AppState state;
  char line[24];
  /* UI 只读快照，不直接修改控制参数或硬件。 */
  AppConfig_GetSnapshot(&config);
  AppState_GetSnapshot(&state);
  OLED_NewFrame();

  switch (g_page)
  {
    case UI_PAGE_STEPPER:
      DrawHeader("步进");
      DrawLine(16, state.stepper_enabled ? "状态:运行" : "状态:停止");
      DrawLine(32, state.stepper_direction_reverse ? "方向:反转" : "方向:正转");
      (void)snprintf(line, sizeof(line), "脉冲:%u", (unsigned)state.stepper_pulse);
      DrawLine(48, line);
      break;

    case UI_PAGE_MOTOR_SPEED:
      DrawHeader("减速电机");
      (void)snprintf(line, sizeof(line), "M1 G:%u/%u %s",
                     (unsigned)state.dc_test_gear,
                     (unsigned)((config.motor_test_pwm_limit + config.motor_test_pwm_step - 1U) /
                                config.motor_test_pwm_step),
                     state.dc_test_direction_reverse ? "REV" : "FWD");
      DrawAsciiLine(18, line);
      (void)snprintf(line, sizeof(line), "PWM:%+d T:%u%%",
                     (int)state.pwm_command[0], (unsigned)state.dc_test_pwm_target);
      DrawAsciiLine(30, line);
      (void)snprintf(line, sizeof(line), "RPM:%+d", RoundedInt(state.measured_rpm[0]));
      DrawAsciiLine(42, line);
      (void)snprintf(line, sizeof(line), "CNT:%ld", (long)state.encoder_count[0]);
      DrawAsciiLine(54, line);
      break;

    case UI_PAGE_MOTOR_TUNING:
      DrawHeader("闭环控制");
      (void)snprintf(line, sizeof(line), "T:%+d R:%+d",
                     RoundedInt(state.target_rpm[0]), RoundedInt(state.measured_rpm[0]));
      DrawAsciiLine(18, line);
      (void)snprintf(line, sizeof(line), "PWM:%+d CNT:%ld",
                     (int)state.pwm_command[0], (long)state.encoder_count[0]);
      DrawAsciiLine(30, line);
      (void)snprintf(line, sizeof(line), "Kp:%d.%02d Ki:%d.%02d",
                     (int)config.speed_kp[0], ((int)(config.speed_kp[0] * 100.0f)) % 100,
                     (int)config.speed_ki[0], ((int)(config.speed_ki[0] * 100.0f)) % 100);
      DrawAsciiLine(42, line);
      (void)snprintf(line, sizeof(line), "FF:%d.%02d SY:%d.%02d",
                     (int)config.speed_feedforward[0], ((int)(config.speed_feedforward[0] * 100.0f)) % 100,
                     (int)config.speed_sync_kp, ((int)(config.speed_sync_kp * 100.0f)) % 100);
      DrawAsciiLine(54, line);
      /* CPR和控制周期继续保留在配置代码中，避免挤占实时观测区域。 */
      (void)config.encoder_counts_per_output_rev;
      (void)config.motor_control_period_ms;
      break;

    case UI_PAGE_ENCODER:
      DrawHeader("编码器");
      for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
      {
        (void)snprintf(line, sizeof(line), "M%u CNT:%ld", (unsigned)(i + 1U),
                       (long)state.encoder_count[i]);
        DrawAsciiLine((uint8_t)(18U + i * 11U), line);
      }
      break;

    case UI_PAGE_SYSTEM:
      DrawHeader("系统");
      DrawLine(16, state.estop_active ? "急停:触发" : "急停:正常");
      DrawLine(32, state.k230_online ? "K230:在线" : "K230:离线");
      (void)snprintf(line, sizeof(line), "FAULT:%08lX", (unsigned long)state.fault_flags);
      DrawAsciiLine(48, line);
      break;

    case UI_PAGE_OVERVIEW:
    default:
      DrawHeader("总览");
      DrawLine(16, state.run_enabled ? "运行:启动" : "运行:停止");
      DrawLine(32, state.mode == APP_MODE_AUTO ? "模式:自动" : "模式:手动");
      DrawLine(48, state.estop_active ? "安全:急停" : "安全:正常");
      break;
  }
  OLED_ShowFrame();
}
