#include "ui_manager.h"

#include "app_state.h"
#include "chassis_motion.h"
#include "font.h"
#include "oled.h"
#include "cmsis_os2.h"
#include "infrared_remote.h"
#include "k230_link.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include <stdio.h>

#define OLED_I2C_ADDRESS 0x78U
#define K230_DISPLAY_WIDTH 640U

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

static void DrawHeader(const char *title)
{
  char page[8];
  OLED_PrintString(0, 0, (char *)title, &font16x16, OLED_COLOR_NORMAL);
  (void)snprintf(page, sizeof(page), "%u/%u", (unsigned)(g_page + 1U),
                 (unsigned)UI_PAGE_COUNT);
  OLED_PrintASCIIString(96, 0, page, &afont12x6, OLED_COLOR_NORMAL);
}

static const char *ControlTargetText(void)
{
  switch (InfraredRemote_GetSelectedTarget())
  {
    case IR_CONTROL_TARGET_CHASSIS: return "底盘";
    case IR_CONTROL_TARGET_X: return "X轴";
    case IR_CONTROL_TARGET_Z: return "Z轴";
    case IR_CONTROL_TARGET_SERVO_1: return "旋转";
    case IR_CONTROL_TARGET_SERVO_2: return "夹爪";
    default: return "未选择";
  }
}

static const char *VisionTaskText(uint8_t task)
{
  if (task == K230_TASK_NUMBER) return "数字";
  if (task == K230_TASK_BEAN) return "豆子";
  return "--";
}

static uint8_t VisionTargetSlot(uint16_t center_x, uint8_t slot_count)
{
  uint32_t slot = ((uint32_t)center_x * slot_count) / K230_DISPLAY_WIDTH;
  if (slot >= slot_count) slot = slot_count - 1U;
  return (uint8_t)slot;
}

static void DrawNumberVisionLayout(const K230VisionResult *vision)
{
  const uint8_t slot_count = 5U;
  uint8_t values[5] = {0U};
  uint8_t confidence[5] = {0U};

  for (uint8_t i = 0U; i < vision->count; ++i)
  {
    const K230VisionTarget *target = &vision->targets[i];
    if ((target->semantic < K230_SEMANTIC_NUMBER_1) ||
        (target->semantic > K230_SEMANTIC_NUMBER_5)) continue;

    uint8_t slot = VisionTargetSlot(target->center_x, slot_count);
    if ((values[slot] == 0U) || (target->confidence_percent > confidence[slot]))
    {
      values[slot] = target->semantic;
      confidence[slot] = target->confidence_percent;
    }
  }

  for (uint8_t slot = 0U; slot < slot_count; ++slot)
  {
    uint8_t x = (uint8_t)(1U + slot * 25U);
    char value[2] = {'-', '\0'};
    if (values[slot] != 0U) value[0] = (char)('0' + values[slot]);
    OLED_DrawRectangle(x, 21U, 23U, 27U, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString((uint8_t)(x + 8U), 29U, value, &afont16x8,
                          OLED_COLOR_NORMAL);
  }
}

static const char *BeanSlotText(uint8_t semantic)
{
  switch (semantic)
  {
    case K230_SEMANTIC_BEAN_L: return "绿";
    case K230_SEMANTIC_BEAN_H: return "黄";
    case K230_SEMANTIC_BEAN_B: return "白";
    default: return "-";
  }
}

static void DrawBeanVisionLayout(const K230VisionResult *vision)
{
  const uint8_t slot_count = 3U;
  uint8_t values[3] = {0U};
  uint8_t confidence[3] = {0U};

  for (uint8_t i = 0U; i < vision->count; ++i)
  {
    const K230VisionTarget *target = &vision->targets[i];
    if ((target->semantic != K230_SEMANTIC_BEAN_L) &&
        (target->semantic != K230_SEMANTIC_BEAN_H) &&
        (target->semantic != K230_SEMANTIC_BEAN_B)) continue;

    uint8_t slot = VisionTargetSlot(target->center_x, slot_count);
    if ((values[slot] == 0U) || (target->confidence_percent > confidence[slot]))
    {
      values[slot] = target->semantic;
      confidence[slot] = target->confidence_percent;
    }
  }

  for (uint8_t slot = 0U; slot < slot_count; ++slot)
  {
    uint8_t x = (uint8_t)(1U + slot * 42U);
    OLED_DrawRectangle(x, 21U, 40U, 27U, OLED_COLOR_NORMAL);
    if (values[slot] == 0U)
      OLED_PrintASCIIString((uint8_t)(x + 17U), 29U, "-", &afont16x8,
                            OLED_COLOR_NORMAL);
    else
      OLED_PrintString((uint8_t)(x + 12U), 27U, (char *)BeanSlotText(values[slot]),
                       &font16x16, OLED_COLOR_NORMAL);
  }
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

void UiManager_PreviousPage(void)
{
  UiManager_SetPage((g_page == UI_PAGE_OVERVIEW) ?
                    (UiPage)(UI_PAGE_COUNT - 1U) : (UiPage)(g_page - 1U));
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
  AppState state;
  K230VisionResult vision;
  InfraredMotionState infrared_motion_state;
  int average_rpm;
  char line[32];
  /* UI 只读快照，不直接修改控制参数或硬件。 */
  AppState_GetSnapshot(&state);
  OLED_NewFrame();

  switch (g_page)
  {
    case UI_PAGE_STEPPER:
    {
      StepperAxisId axis = (InfraredRemote_GetSelectedAxis() == IR_CONTROL_AXIS_X) ?
                           STEPPER_AXIS_X : STEPPER_AXIS_Z;
      DrawHeader("步进控制");
      infrared_motion_state = InfraredRemote_GetSelectedMotionState();
      (void)snprintf(line, sizeof(line), "选择:%c轴 %s",
                     (axis == STEPPER_AXIS_X) ? 'X' : 'Z',
                     InfraredRemote_GetSelectedDirectionReverse() ? "反转" : "正转");
      DrawLine(16, line);
      if (InfraredRemote_IsDirectionChangePending()) DrawLine(32, "状态:换向等待");
      else if (infrared_motion_state != IR_MOTION_STOP) DrawLine(32, "状态:运行");
      else if (StepperAxis_IsHolding(axis)) DrawLine(32, "状态:保位");
      else DrawLine(32, "状态:停止");
      (void)snprintf(line, sizeof(line), "脉冲:%ld",
                     (long)StepperAxis_GetPositionPulses(axis));
      DrawLine(48, line);
      break;
    }

    case UI_PAGE_STEPPER_PULSE:
    {
      StepperAxisId axis = (InfraredRemote_GetSelectedAxis() == IR_CONTROL_AXIS_X) ?
                           STEPPER_AXIS_X : STEPPER_AXIS_Z;
      DrawHeader("脉冲控制");
      (void)snprintf(line, sizeof(line), "选择:%c轴 %s",
                     (axis == STEPPER_AXIS_X) ? 'X' : 'Z',
                     InfraredRemote_GetSelectedDirectionReverse() ? "反转" : "正转");
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "目标:%lu",
                     (unsigned long)InfraredRemote_GetPulseInput());
      DrawLine(32, line);
      (void)snprintf(line, sizeof(line), "实际:%lu %s",
                     (unsigned long)StepperAxis_GetCompletedPulses(axis),
                     StepperAxis_IsPulseMoveActive(axis) ? "运行" : "停止");
      DrawLine(48, line);
      break;
    }

    case UI_PAGE_SERVO:
    {
      uint8_t servo = InfraredRemote_GetSelectedServoIndex();
      DrawHeader("舵机控制");
      (void)snprintf(line, sizeof(line), "选择:%s", (servo == 0U) ? "旋转" : "夹爪");
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "角度:%u",
                     (unsigned)ServoControl_GetAngle(servo));
      DrawLine(32, line);
      (void)snprintf(line, sizeof(line), "脉冲:%u",
                     (unsigned)ServoControl_GetPulseUs(servo));
      DrawLine(48, line);
      break;
    }

    case UI_PAGE_MOTOR_SPEED:
      DrawHeader("底盘控制");
      average_rpm = (RoundedInt(state.measured_rpm[0]) +
                     RoundedInt(state.measured_rpm[1]) +
                     RoundedInt(state.measured_rpm[2]) +
                     RoundedInt(state.measured_rpm[3])) / 4;
      DrawLine(16, ChassisMotion_IsRunning() ? "状态:运行" : "状态:停止");
      (void)snprintf(line, sizeof(line), "速度:%d/%d", average_rpm,
                     (int)ChassisMotion_GetTargetRpm());
      DrawLine(32, line);
      DrawLine(48, ChassisMotion_IsDirectionReverse() ?
               "方向:反转" : "方向:正转");
      break;

    case UI_PAGE_VISION:
      DrawHeader((K230Link_GetRequestedTask() == K230_TASK_BEAN) ?
                 "豆子识别" : "数字识别");
      if (K230Link_GetTaskSwitchState() == K230_TASK_SWITCH_PENDING)
      {
        (void)snprintf(line, sizeof(line), "任务:%s",
                       VisionTaskText(K230Link_GetRequestedTask()));
        DrawLine(16, line);
        DrawLine(32, "状态:等待");
        DrawLine(48, "按键:1数字2豆子");
        break;
      }
      if (K230Link_GetTaskSwitchState() == K230_TASK_SWITCH_FAILED)
      {
        DrawLine(16, "状态:失败");
        DrawLine(32, "通信:超时");
        DrawLine(48, "按键:1数字2豆子");
        break;
      }
      if (!state.k230_online || !K230Link_GetLatestResult(&vision))
      {
        DrawLine(16, "状态:离线");
        DrawLine(32, "通信:超时");
        DrawLine(48, "按键:1数字2豆子");
        break;
      }

      if (vision.task == K230_TASK_BEAN) DrawBeanVisionLayout(&vision);
      else DrawNumberVisionLayout(&vision);
      (void)snprintf(line, sizeof(line), "在线 数:%u",
                     (unsigned)vision.count);
      DrawLine(48, line);
      break;

    case UI_PAGE_SYSTEM:
      DrawHeader("系统状态");
      DrawLine(16, state.estop_active ? "急停:触发" : "急停:正常");
      DrawLine(32, state.k230_online ? "视觉:在线" : "视觉:离线");
      (void)snprintf(line, sizeof(line), "故障:%08lX", (unsigned long)state.fault_flags);
      DrawLine(48, line);
      break;

    case UI_PAGE_OVERVIEW:
    default:
      DrawHeader("总览");
      (void)snprintf(line, sizeof(line), "控制:%s", ControlTargetText());
      DrawLine(16, line);
      DrawLine(32, state.estop_active ? "安全:故障" : "安全:正常");
      (void)snprintf(line, sizeof(line), "遥控:%02X",
                     (unsigned)InfraredRemote_GetLastCommand());
      DrawLine(48, line);
      break;
  }
  OLED_ShowFrame();
}
