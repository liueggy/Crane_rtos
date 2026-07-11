#include "ui_manager.h"

#include "app_config.h"
#include "app_state.h"
#include "font.h"
#include "oled.h"
#include <stdio.h>

#define OLED_I2C_ADDRESS 0x78U

/* 当前页面只保存页面编号，具体内容由 Render 根据状态快照绘制。 */
static UiPage g_page = UI_PAGE_OVERVIEW;

static int RoundedInt(float value)
{
  return (int)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static void DrawLine(uint8_t y, const char *text)
{
  OLED_PrintASCIIString(0, y, text, &afont12x6, OLED_COLOR_NORMAL);
}

static void DrawHeader(const char *title)
{
  char line[22];
  /* 统一显示标题和页码，方便后续增加页面。 */
  (void)snprintf(line, sizeof(line), "%s %u/%u", title,
                 (unsigned)(g_page + 1U), (unsigned)UI_PAGE_COUNT);
  OLED_PrintASCIIString(0, 0, line, &afont16x8, OLED_COLOR_NORMAL);
}

void UiManager_Init(I2C_HandleTypeDef *i2c)
{
  OLED_Init(i2c, OLED_I2C_ADDRESS);
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
    case UI_PAGE_MOTOR_SPEED:
      DrawHeader("SPEED");
      for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
      {
        (void)snprintf(line, sizeof(line), "M%u %4d/%4d P%3d", (unsigned)(i + 1U),
                       RoundedInt(state.measured_rpm[i]), RoundedInt(state.target_rpm[i]),
                       (int)state.pwm_command[i]);
        DrawLine((uint8_t)(18U + i * 11U), line);
      }
      break;

    case UI_PAGE_MOTOR_TUNING:
      DrawHeader("TUNE");
      (void)snprintf(line, sizeof(line), "Kp:%d.%02d Ki:%d.%02d",
                     (int)config.speed_kp[0], ((int)(config.speed_kp[0] * 100.0f)) % 100,
                     (int)config.speed_ki[0], ((int)(config.speed_ki[0] * 100.0f)) % 100);
      DrawLine(18, line);
      (void)snprintf(line, sizeof(line), "FF:%d.%02d SY:%d.%02d",
                     (int)config.speed_feedforward[0], ((int)(config.speed_feedforward[0] * 100.0f)) % 100,
                     (int)config.speed_sync_kp, ((int)(config.speed_sync_kp * 100.0f)) % 100);
      DrawLine(30, line);
      (void)snprintf(line, sizeof(line), "ACC:%d DEC:%d", RoundedInt(config.acceleration_rpm_s),
                     RoundedInt(config.deceleration_rpm_s));
      DrawLine(42, line);
      (void)snprintf(line, sizeof(line), "CPR:%d DT:%ums", RoundedInt(config.encoder_counts_per_output_rev),
                     (unsigned)config.motor_control_period_ms);
      DrawLine(54, line);
      break;

    case UI_PAGE_ENCODER:
      DrawHeader("ENCODER");
      for (uint8_t i = 0U; i < APP_MOTOR_COUNT; ++i)
      {
        (void)snprintf(line, sizeof(line), "M%u CNT:%ld", (unsigned)(i + 1U),
                       (long)state.encoder_count[i]);
        DrawLine((uint8_t)(18U + i * 11U), line);
      }
      break;

    case UI_PAGE_SYSTEM:
      DrawHeader("SYSTEM");
      DrawLine(18, state.estop_active ? "ESTOP: ACTIVE" : "ESTOP: OK");
      DrawLine(30, state.k230_online ? "K230: ONLINE" : "K230: OFFLINE");
      (void)snprintf(line, sizeof(line), "FAULT:%08lX", (unsigned long)state.fault_flags);
      DrawLine(42, line);
      DrawLine(54, "Hold K1: Next");
      break;

    case UI_PAGE_OVERVIEW:
    default:
      DrawHeader("CRANE");
      DrawLine(18, state.run_enabled ? "State: RUN" : "State: STOP");
      DrawLine(30, state.mode == APP_MODE_AUTO ? "Mode: AUTO" : "Mode: MANUAL");
      DrawLine(42, state.estop_active ? "Safety: ESTOP" : "Safety: READY");
      DrawLine(54, "K0 Run K1 Dir");
      break;
  }
  OLED_ShowFrame();
}
