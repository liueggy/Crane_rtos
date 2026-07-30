#include "ui_manager.h"

#include "app_state.h"
#include "bean_pickup_demo.h"
#include "bean_sequence_demo.h"
#include "box_calibration.h"
#include "chassis_motion.h"
#include "drop_demo.h"
#include "font.h"
#include "oled.h"
#include "odometry_calibration.h"
#include "photo_sensor.h"
#include "route_executor.h"
#include "robot_controller.h"
#include "cmsis_os2.h"
#include "infrared_remote.h"
#include "initialization_debug.h"
#include "k230_link.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "z_calibration.h"
#include "vision_route_demo.h"
#include "xy_waypoint_demo.h"
#include "world_map.h"
#include <stdio.h>
#include <string.h>

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

static const char *DropDemoStateText(DropDemoState state)
{
  switch (state)
  {
    case DROP_DEMO_NEED_REFERENCE: return "待机";
    case DROP_DEMO_LIFT_SAFE:
    case DROP_DEMO_RAISE: return "上升";
    case DROP_DEMO_MOVE_X: return "移动";
    case DROP_DEMO_ROTATE: return "旋转";
    case DROP_DEMO_DESCEND: return "下降";
    case DROP_DEMO_RELEASE: return "放豆";
    case DROP_DEMO_RESET_TOOL: return "复位";
    case DROP_DEMO_PAUSED: return "停止";
    case DROP_DEMO_COMPLETE: return "完成";
    case DROP_DEMO_FAULT: return "故障";
    case DROP_DEMO_IDLE:
    default: return "待机";
  }
}

static const char *BeanPickupDemoStateText(BeanPickupDemoState state)
{
  switch (state)
  {
    case BEAN_PICKUP_DEMO_HOME_Z_SETTLE:
    case BEAN_PICKUP_DEMO_HOME_X_SETTLE:
    case BEAN_PICKUP_DEMO_CLEAR_X_SETTLE:
    case BEAN_PICKUP_DEMO_PREPARE_Z_SETTLE:
    case BEAN_PICKUP_DEMO_MOVE_X_SETTLE:
    case BEAN_PICKUP_DEMO_RAISE_SETTLE: return "换向等待";
    case BEAN_PICKUP_DEMO_HOME_Z_TOP: return "Z回顶";
    case BEAN_PICKUP_DEMO_HOME_X_RIGHT: return "X回零";
    case BEAN_PICKUP_DEMO_CLEAR_X_LIMIT: return "X脱离";
    case BEAN_PICKUP_DEMO_READY: return "READY";
    case BEAN_PICKUP_DEMO_PREPARE_Z: return "最高位置";
    case BEAN_PICKUP_DEMO_MOVE_X: return "移到X";
    case BEAN_PICKUP_DEMO_OPEN_GRIPPER: return "张开";
    case BEAN_PICKUP_DEMO_DESCEND: return "下降";
    case BEAN_PICKUP_DEMO_CLOSE_GRIPPER: return "闭合";
    case BEAN_PICKUP_DEMO_RAISE: return "上升";
    case BEAN_PICKUP_DEMO_COMPLETE: return "完成";
    case BEAN_PICKUP_DEMO_FAULT: return "故障";
    case BEAN_PICKUP_DEMO_IDLE:
    default: return "待机";
  }
}

static const char *BeanSequenceDemoStateText(BeanSequenceDemoState state)
{
  switch (state)
  {
    case BEAN_SEQUENCE_DEMO_REFERENCE: return "XZ回零";
    case BEAN_SEQUENCE_DEMO_MOVE_TO_B: return "前往B";
    case BEAN_SEQUENCE_DEMO_PICK_B: return "抓B";
    case BEAN_SEQUENCE_DEMO_RELEASE_B: return "B释放";
    case BEAN_SEQUENCE_DEMO_MOVE_TO_AC: return "前往AC";
    case BEAN_SEQUENCE_DEMO_PICK_C: return "抓C";
    case BEAN_SEQUENCE_DEMO_RELEASE_C: return "C释放";
    case BEAN_SEQUENCE_DEMO_PICK_A: return "抓A";
    case BEAN_SEQUENCE_DEMO_COMPLETE: return "完成";
    case BEAN_SEQUENCE_DEMO_FAULT: return "故障";
    case BEAN_SEQUENCE_DEMO_IDLE:
    default: return "待机";
  }
}

static const char *XyWaypointDemoStateText(XyWaypointDemoState state)
{
  switch (state)
  {
    case XY_WAYPOINT_DEMO_REFERENCE: return "XZ回零";
    case XY_WAYPOINT_DEMO_READY: return "READY";
    case XY_WAYPOINT_DEMO_MOVE_Y: return "移动Y";
    case XY_WAYPOINT_DEMO_CENTER_X_SETTLE: return "起点换边";
    case XY_WAYPOINT_DEMO_CENTER_X_MOVE: return "起点横移";
    case XY_WAYPOINT_DEMO_X_SETTLE: return "换向等待";
    case XY_WAYPOINT_DEMO_MOVE_X: return "移动X";
    case XY_WAYPOINT_DEMO_COMPLETE: return "到达";
    case XY_WAYPOINT_DEMO_FAULT: return "故障";
    case XY_WAYPOINT_DEMO_IDLE:
    default: return "待机";
  }
}

static const char *InitializationDebugStateText(InitializationDebugState state)
{
  switch (state)
  {
    case INITIALIZATION_DEBUG_SEEK_Z_BOTTOM: return "触底";
    case INITIALIZATION_DEBUG_DIRECTION_SETTLE: return "换向等待";
    case INITIALIZATION_DEBUG_RAISE_Z_TOP: return "上升";
    case INITIALIZATION_DEBUG_HOME_X_RIGHT: return "X回零";
    case INITIALIZATION_DEBUG_MOVE_X_CENTER: return "X移中";
    case INITIALIZATION_DEBUG_LOWER_Z_BOTTOM: return "下降";
    case INITIALIZATION_DEBUG_COMPLETE: return "完成";
    case INITIALIZATION_DEBUG_FAULT: return "故障";
    case INITIALIZATION_DEBUG_IDLE:
    default: return "待机";
  }
}

static const char *OdometryCalibrationStateText(OdometryCalibrationState state)
{
  switch (state)
  {
    case ODOMETRY_CALIBRATION_RUNNING: return "运行";
    case ODOMETRY_CALIBRATION_COMPLETE: return "完成";
    case ODOMETRY_CALIBRATION_STOPPED: return "停止";
    case ODOMETRY_CALIBRATION_FAULT: return "故障";
    case ODOMETRY_CALIBRATION_IDLE:
    default: return "待机";
  }
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

static const char *VisionRouteDemoStateText(VisionRouteDemoState state)
{
  switch (state)
  {
    case VISION_ROUTE_DEMO_LIFT_Z: return "上升";
    case VISION_ROUTE_DEMO_HOME_X: return "X回零";
    case VISION_ROUTE_DEMO_ALIGN_X: return "X移动";
    case VISION_ROUTE_DEMO_MOVE_NUMBER: return "数字移动";
    case VISION_ROUTE_DEMO_SCAN_NUMBER_START:
    case VISION_ROUTE_DEMO_SCAN_NUMBER_ROW:
    case VISION_ROUTE_DEMO_SCAN_NUMBER_SIDE: return "数字识别";
    case VISION_ROUTE_DEMO_RESCAN_NUMBER_MOVE:
    case VISION_ROUTE_DEMO_RESCAN_NUMBER_DWELL:
    case VISION_ROUTE_DEMO_RESCAN_NUMBER_SIDE: return "数字补扫";
    case VISION_ROUTE_DEMO_MOVE_BEAN: return "豆子移动";
    case VISION_ROUTE_DEMO_SCAN_BEAN_A:
    case VISION_ROUTE_DEMO_SCAN_BEAN_B: return "豆子识别";
    case VISION_ROUTE_DEMO_COMPLETE: return "完成";
    case VISION_ROUTE_DEMO_FAULT: return "故障";
    case VISION_ROUTE_DEMO_IDLE:
    default: return "待机";
  }
}

static uint8_t VisionTargetSlot(uint16_t center_x, uint8_t slot_count)
{
  uint32_t slot = ((uint32_t)center_x * slot_count) / K230_DISPLAY_WIDTH;
  if (slot >= slot_count) slot = slot_count - 1U;
  return (uint8_t)slot;
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

static const char *RobotFaultText(RobotFaultCode fault)
{
  switch (fault)
  {
    case ROBOT_FAULT_ESTOP: return "急停";
    case ROBOT_FAULT_START_POSE: return "起点姿态";
    case ROBOT_FAULT_CALIBRATION: return "标定";
    case ROBOT_FAULT_VISION: return "识别";
    case ROBOT_FAULT_TASK_MAP: return "任务映射";
    case ROBOT_FAULT_NAVIGATION: return "光电导航";
    case ROBOT_FAULT_ACTION: return "机构动作";
    case ROBOT_FAULT_X_COORDINATE: return "X坐标";
    case ROBOT_FAULT_Z_COORDINATE: return "Z坐标";
    case ROBOT_FAULT_FINAL_HOME: return "回起点";
    case ROBOT_FAULT_NONE:
    default: return "无";
  }
}

static void FormatNumberVisionLine(const K230VisionResult *vision,
                                   char *text, size_t size)
{
  uint8_t values[5] = {0U};
  uint8_t confidence[5] = {0U};
  char shown[5] = {'-', '-', '-', '-', '-'};
  for (uint8_t i = 0U; i < vision->count; ++i)
  {
    const K230VisionTarget *target = &vision->targets[i];
    if ((target->semantic < K230_SEMANTIC_NUMBER_1) ||
        (target->semantic > K230_SEMANTIC_NUMBER_5)) continue;
    uint8_t slot = VisionTargetSlot(target->center_x, 5U);
    if ((values[slot] == 0U) ||
        (target->confidence_percent > confidence[slot]))
    {
      values[slot] = target->semantic;
      confidence[slot] = target->confidence_percent;
      shown[slot] = (char)('0' + target->semantic);
    }
  }
  (void)snprintf(text, size, "数:%c %c %c %c %c",
                 shown[0], shown[1], shown[2], shown[3], shown[4]);
}

static void FormatBeanVisionLine(const K230VisionResult *vision,
                                 char *text, size_t size)
{
  uint8_t values[3] = {0U};
  uint8_t confidence[3] = {0U};
  for (uint8_t i = 0U; i < vision->count; ++i)
  {
    const K230VisionTarget *target = &vision->targets[i];
    if ((target->semantic != K230_SEMANTIC_BEAN_L) &&
        (target->semantic != K230_SEMANTIC_BEAN_H) &&
        (target->semantic != K230_SEMANTIC_BEAN_B)) continue;
    uint8_t slot = VisionTargetSlot(target->center_x, 3U);
    if ((values[slot] == 0U) ||
        (target->confidence_percent > confidence[slot]))
    {
      values[slot] = target->semantic;
      confidence[slot] = target->confidence_percent;
    }
  }
  (void)snprintf(text, size, "豆:%s %s %s", BeanSlotText(values[0]),
                 BeanSlotText(values[1]), BeanSlotText(values[2]));
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

    case UI_PAGE_BOX_CALIBRATION:
    {
      uint8_t slot = BoxCalibration_GetSelectedSlot();
      DrawHeader("箱位标定");
      (void)snprintf(line, sizeof(line), "箱位:%u %s",
                     (unsigned)(slot + 1U), (slot < 3U) ? "底" : "侧");
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "X:%ld 步:%lu",
                     (long)BoxCalibration_GetCurrentX(),
                     (unsigned long)BoxCalibration_GetJogStep());
      DrawLine(32, line);
      (void)snprintf(line, sizeof(line), "零:%s 存:%s",
                     BoxCalibration_IsReferenceValid() ? "已" : "未",
                     BoxCalibration_IsSelectedSaved() ? "已" : "未");
      DrawLine(48, line);
      break;
    }

    case UI_PAGE_Z_CALIBRATION:
    {
      uint8_t target = ZCalibration_GetSelectedTarget();
      DrawHeader("Z高度标定");
      if (target < Z_CALIBRATION_DROP)
        (void)snprintf(line, sizeof(line), "夹取:%u",
                       (unsigned)(target + 1U));
      else
        (void)snprintf(line, sizeof(line), "放豆");
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "Z:%ld 步:%lu",
                     (long)ZCalibration_GetCurrentZ(),
                     (unsigned long)ZCalibration_GetJogStep());
      DrawLine(32, line);
      (void)snprintf(line, sizeof(line), "底:%s 存:%s",
                     StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z) ? "上升" :
                     (ZCalibration_IsReferenceValid() ? "已" : "未"),
                     ZCalibration_IsSelectedSaved() ? "已" : "未");
      DrawLine(48, line);
      break;
    }

    case UI_PAGE_INITIALIZATION_DEBUG:
      DrawHeader("回零控制");
      (void)snprintf(line, sizeof(line), "状态:%s",
                     InitializationDebugStateText(InitializationDebug_GetState()));
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "X:%ld Z:%ld",
                     (long)StepperAxis_GetPositionPulses(STEPPER_AXIS_X),
                     (long)StepperAxis_GetPositionPulses(STEPPER_AXIS_Z));
      DrawLine(32, line);
      DrawLine(48, "电源:启动 0:停止");
      break;

    case UI_PAGE_DROP_DEMO:
      DrawHeader("放豆控制");
      (void)snprintf(line, sizeof(line), "箱位:%u 状态:%s",
                     (unsigned)DropDemo_GetSlotNumber(),
                     DropDemoStateText(DropDemo_GetState()));
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "X:%ld Z:%ld",
                     (long)StepperAxis_GetPositionPulses(STEPPER_AXIS_X),
                     (long)StepperAxis_GetPositionPulses(STEPPER_AXIS_Z));
      DrawLine(32, line);
      DrawLine(48, "电源:启停 0:复位");
      break;

    case UI_PAGE_BEAN_PICKUP_DEMO:
    {
      uint8_t position = BeanPickupDemo_GetSelectedPosition();
      static const int32_t x_positions[] = {
        BEAN_SLOT_TOP_LEFT_X_PULSES,
        BEAN_SLOT_OFFSET_X_PULSES,
        BEAN_SLOT_TOP_RIGHT_X_PULSES,
      };
      static const int32_t z_positions[] = {
        BEAN_PICKUP_Z_LEVEL_3_PULSES,
        BEAN_PICKUP_Z_LEVEL_1_PULSES,
        BEAN_PICKUP_Z_LEVEL_2_PULSES,
      };
      static const char *const position_names[] = {"左", "中", "右"};
      DrawHeader("抓豆控制");
      (void)snprintf(line, sizeof(line), "位置:%u%s 状态:%s",
                     (unsigned)(position + 1U), position_names[position],
                     BeanPickupDemoStateText(BeanPickupDemo_GetState()));
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "目标 X:%ld Z:%ld",
                     (long)x_positions[position], (long)z_positions[position]);
      DrawLine(32, line);
      DrawLine(48, "1/2/3选 电源:启动");
      break;
    }

    case UI_PAGE_BEAN_SEQUENCE_DEMO:
      DrawHeader("B-C-A抓豆");
      (void)snprintf(line, sizeof(line), "状态:%s 速度:%d",
                     BeanSequenceDemoStateText(BeanSequenceDemo_GetState()),
                     (int)ChassisMotion_GetTargetRpm());
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "挡板:%u/%u X:%ld",
                     (unsigned)ChassisMotion_GetPassedLandmarkCount(
                         CHASSIS_SIDE_ORIGIN),
                     (unsigned)ChassisMotion_GetPassedLandmarkCount(
                         CHASSIS_SIDE_FAR),
                     (long)StepperAxis_GetPositionPulses(STEPPER_AXIS_X));
      DrawLine(32, line);
      DrawLine(48, "电源:启动 0:停止");
      break;

    case UI_PAGE_XY_WAYPOINT_DEMO:
      DrawHeader("XY点位导航");
      (void)snprintf(line, sizeof(line), "当前:%s 目标:%s",
                     XyWaypointDemo_GetCurrentName(),
                     XyWaypointDemo_GetName(XyWaypointDemo_GetSelected()));
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "状态:%s X:%ld",
                     XyWaypointDemoStateText(XyWaypointDemo_GetState()),
                     (long)StepperAxis_GetPositionPulses(STEPPER_AXIS_X));
      DrawLine(32, line);
      DrawLine(48, "1-5箱 7A 8B 9C 电源走");
      break;

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
    {
      uint8_t side_mask = ChassisMotion_GetRunningSideMask();
      uint8_t armed_mask = ChassisMotion_IsPhotoStopArmed();
      DrawHeader("底盘控制");
      average_rpm = (RoundedInt(state.measured_rpm[0]) +
                     RoundedInt(state.measured_rpm[1]) +
                     RoundedInt(state.measured_rpm[2]) +
                     RoundedInt(state.measured_rpm[3])) / 4;
      (void)snprintf(line, sizeof(line), "M12:%u M34:%u %s",
                     (unsigned)((side_mask & 1U) ? 1U : 0U),
                     (unsigned)((side_mask & 2U) ? 1U : 0U),
                     ChassisMotion_IsDirectionReverse() ? "反" : "正");
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "原:%u 远:%u 防:%u%u",
                     (unsigned)PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_ORIGIN_SIDE),
                     (unsigned)PhotoSensor_GetState(PHOTO_SENSOR_CHASSIS_FAR_SIDE),
                     (unsigned)((armed_mask & 1U) ? 1U : 0U),
                     (unsigned)((armed_mask & 2U) ? 1U : 0U));
      DrawLine(32, line);
      (void)snprintf(line, sizeof(line), "速:%d/%d 时:%u",
                     average_rpm, (int)ChassisMotion_GetTargetRpm(),
                     (unsigned)ChassisMotion_GetAlignTimeoutMs());
      DrawLine(48, line);
      break;
    }

    case UI_PAGE_ODOMETRY_CALIBRATION:
    {
      uint32_t measured = OdometryCalibration_GetMeasuredDistanceMmX10();
      uint32_t spread = OdometryCalibration_GetWheelSpreadMmX10();
      OdometryCalibrationField field = OdometryCalibration_GetSelectedField();
      OdometryCalibrationState calibration_state = OdometryCalibration_GetState();
      DrawHeader("距离标定");
      (void)snprintf(line, sizeof(line), "%c距:%u %c速:%u",
                     (field == ODOMETRY_FIELD_DISTANCE) ? '>' : ' ',
                     (unsigned)OdometryCalibration_GetTargetDistanceMm(),
                     (field == ODOMETRY_FIELD_SPEED) ? '>' : ' ',
                     (unsigned)OdometryCalibration_GetTargetSpeedRpm());
      DrawLine(16, line);
      if ((calibration_state == ODOMETRY_CALIBRATION_RUNNING) ||
          (calibration_state == ODOMETRY_CALIBRATION_COMPLETE) ||
          (calibration_state == ODOMETRY_CALIBRATION_STOPPED))
      {
        (void)snprintf(line, sizeof(line), "1:%lu 2:%lu %s",
                       (unsigned long)((OdometryCalibration_GetWheelDistanceMmX10(0U) + 5U) / 10U),
                       (unsigned long)((OdometryCalibration_GetWheelDistanceMmX10(1U) + 5U) / 10U),
                       OdometryCalibrationStateText(calibration_state));
        DrawLine(32, line);
        (void)snprintf(line, sizeof(line), "3:%lu 4:%lu",
                       (unsigned long)((OdometryCalibration_GetWheelDistanceMmX10(2U) + 5U) / 10U),
                       (unsigned long)((OdometryCalibration_GetWheelDistanceMmX10(3U) + 5U) / 10U));
        DrawLine(48, line);
      }
      else
      {
        (void)snprintf(line, sizeof(line), "%s 状:%s",
                       OdometryCalibration_GetDirectionReverse() ? "反" : "正",
                       OdometryCalibrationStateText(calibration_state));
        DrawLine(32, line);
        (void)snprintf(line, sizeof(line), "计:%lu 差:%lu",
                       (unsigned long)((measured + 5U) / 10U),
                       (unsigned long)((spread + 5U) / 10U));
        DrawLine(48, line);
      }
      break;
    }

    case UI_PAGE_VISION:
    {
      VisionRouteDemoState demo_state = VisionRouteDemo_GetState();
      K230VisionResult number_result;
      K230VisionResult bean_result;
      K230VisionResult live_result;
      memset(&number_result, 0, sizeof(number_result));
      memset(&bean_result, 0, sizeof(bean_result));
      memset(&live_result, 0, sizeof(live_result));
      number_result.task = K230_TASK_NUMBER;
      bean_result.task = K230_TASK_BEAN;

      DrawHeader("视觉识别");
      if (demo_state == VISION_ROUTE_DEMO_IDLE)
      {
        K230TaskSwitchState switch_state = K230Link_GetTaskSwitchState();
        K230VisionTask shown_task = (switch_state == K230_TASK_SWITCH_PENDING) ?
                                     K230Link_GetRequestedTask() :
                                     K230Link_GetSelectedTask();
        char received = '-';
        DrawLine(16, "1:数字 2:豆子");
        if (switch_state == K230_TASK_SWITCH_PENDING)
          (void)snprintf(line, sizeof(line), "模式:切%s",
                         (shown_task == K230_TASK_BEAN) ? "豆子" : "数字");
        else if (switch_state == K230_TASK_SWITCH_FAILED)
          (void)snprintf(line, sizeof(line), "模式:切换失败");
        else
          (void)snprintf(line, sizeof(line), "模式:%s%s",
                         (shown_task == K230_TASK_BEAN) ? "豆子" : "数字",
                         state.k230_online ? "" : " 离线");
        DrawLine(32, line);

        if (K230Link_GetLatestResult(&live_result) && live_result.valid &&
            (live_result.count > 0U) && K230Link_IsResultFresh(1000U))
        {
          uint8_t semantic = live_result.targets[0].semantic;
          if ((semantic >= K230_SEMANTIC_NUMBER_1) &&
              (semantic <= K230_SEMANTIC_NUMBER_5))
            received = (char)('0' + semantic);
          else if (semantic == K230_SEMANTIC_BEAN_L) received = 'L';
          else if (semantic == K230_SEMANTIC_BEAN_H) received = 'H';
          else if (semantic == K230_SEMANTIC_BEAN_B) received = 'B';
        }
        (void)snprintf(line, sizeof(line), "接收:%c", received);
        DrawLine(48, line);
        break;
      }

      (void)VisionRouteDemo_GetDisplayResult(&number_result);
      (void)VisionRouteDemo_GetDisplayResult(&bean_result);
      FormatNumberVisionLine(&number_result, line, sizeof(line));
      DrawLine(16, line);
      FormatBeanVisionLine(&bean_result, line, sizeof(line));
      DrawLine(32, line);
      (void)snprintf(line, sizeof(line), "状态:%s%s",
                     VisionRouteDemoStateText(demo_state),
                     state.k230_online ? "" : " 离线");
      DrawLine(48, line);
      break;
    }

    case UI_PAGE_COMPETITION:
    {
      const MissionTransportTask *task = RobotController_GetActiveTask();
      const WorldPose *pose = WorldMap_GetPose();
      uint8_t task_index = RobotController_GetTaskIndex();
      DrawHeader("比赛运行");
      if (task != NULL)
        (void)snprintf(line, sizeof(line), "任务:%u/3 %c->数字%u",
                       (unsigned)(task_index + 1U),
                       (char)('A' + task_index),
                       (unsigned)task->target_number);
      else
        (void)snprintf(line, sizeof(line), "任务:%u/3 待识别",
                       (unsigned)(task_index + 1U));
      DrawLine(16, line);
      (void)snprintf(line, sizeof(line), "阶段:%s S%u>S%u",
                     RobotController_GetPhaseText(),
                     (unsigned)RobotController_GetCurrentStation(),
                     (unsigned)RobotController_GetTargetStation());
      DrawLine(32, line);
      if (RobotController_GetState() == ROBOT_STATE_FAULT)
        (void)snprintf(line, sizeof(line), "故障:%s",
                       RobotFaultText(RobotController_GetFaultCode()));
      else
        (void)snprintf(line, sizeof(line), "坐标:X%c Y%c Z%c PWR/0",
                       pose->x_valid ? '+' : '-', pose->y_valid ? '+' : '-',
                       pose->z_valid ? '+' : '-');
      DrawLine(48, line);
      break;
    }

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
