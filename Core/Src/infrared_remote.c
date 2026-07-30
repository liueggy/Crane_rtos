#include "infrared_remote.h"

#include "main.h"
#include "bean_pickup_demo.h"
#include "bean_sequence_demo.h"
#include "box_calibration.h"
#include "buzzer.h"
#include "chassis_motion.h"
#include "drop_demo.h"
#include "initialization_debug.h"
#include "k230_link.h"
#include "odometry_calibration.h"
#include "photo_sensor.h"
#include "robot_controller.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "ui_manager.h"
#include "z_calibration.h"
#include "vision_route_demo.h"
#include "xy_waypoint_demo.h"

/* TIM4 的 1 MHz 计数单位为微秒。以下窗口保留了接收头误差余量。 */
#define IR_LEADER_MARK_MIN_US 8000U
#define IR_LEADER_MARK_MAX_US 10000U
#define IR_LEADER_SPACE_MIN_US 3500U
#define IR_LEADER_SPACE_MAX_US 5500U
#define IR_BIT_MARK_MIN_US 350U
#define IR_BIT_MARK_MAX_US 850U
#define IR_BIT_ZERO_SPACE_MIN_US 350U
#define IR_BIT_ZERO_SPACE_MAX_US 900U
#define IR_BIT_ONE_SPACE_MIN_US 1300U
#define IR_BIT_ONE_SPACE_MAX_US 2100U
#define IR_FRAME_BITS 32U
#define IR_FRAME_TIMEOUT_MS 30U
#define IR_X_DIRECTION_CHANGE_DELAY_MS 300U
#define IR_KEY_BEEP_DURATION_MS 60U

typedef enum
{
  IR_WAIT_LEADER_MARK = 0,
  IR_WAIT_LEADER_SPACE,
  IR_WAIT_BIT_MARK,
  IR_WAIT_BIT_SPACE
} InfraredRemoteDecodeState;

static TIM_HandleTypeDef *g_ir_timer;
static volatile uint16_t g_last_capture;
static volatile uint32_t g_frame;
static volatile uint8_t g_bit_index;
static volatile uint8_t g_capture_started;
static volatile uint32_t g_last_edge_tick;
static volatile InfraredRemoteDecodeState g_decode_state;
static volatile uint8_t g_pending_command;
static volatile uint8_t g_command_ready;
static uint8_t g_last_command;
static volatile InfraredControlAxis g_selected_axis;
static volatile InfraredControlTarget g_selected_target;
static volatile InfraredMotionState g_motion_state[IR_CONTROL_AXIS_COUNT];
static InfraredMotionState g_selected_direction[IR_CONTROL_AXIS_COUNT];
static uint8_t g_direction_change_pending[IR_CONTROL_AXIS_COUNT];
static uint32_t g_direction_resume_tick[IR_CONTROL_AXIS_COUNT];
static uint32_t g_pulse_input;

static uint8_t InfraredRemote_IsKnownCommand(uint8_t command)
{
  switch (command)
  {
    case IR_REMOTE_CMD_POWER:
    case IR_REMOTE_CMD_UP:
    case IR_REMOTE_CMD_DOWN:
    case IR_REMOTE_CMD_RIGHT:
    case IR_REMOTE_CMD_LEFT:
    case IR_REMOTE_CMD_0:
    case IR_REMOTE_CMD_1:
    case IR_REMOTE_CMD_2:
    case IR_REMOTE_CMD_3:
    case IR_REMOTE_CMD_4:
    case IR_REMOTE_CMD_5:
    case IR_REMOTE_CMD_6:
    case IR_REMOTE_CMD_7:
    case IR_REMOTE_CMD_8:
    case IR_REMOTE_CMD_9:
    case IR_REMOTE_CMD_VOL_MINUS:
    case IR_REMOTE_CMD_VOL_PLUS:
      return 1U;
    default:
      return 0U;
  }
}

static int8_t InfraredRemote_DigitValue(uint8_t command)
{
  switch (command)
  {
    case IR_REMOTE_CMD_0: return 0;
    case IR_REMOTE_CMD_1: return 1;
    case IR_REMOTE_CMD_2: return 2;
    case IR_REMOTE_CMD_3: return 3;
    case IR_REMOTE_CMD_4: return 4;
    case IR_REMOTE_CMD_5: return 5;
    case IR_REMOTE_CMD_6: return 6;
    case IR_REMOTE_CMD_7: return 7;
    case IR_REMOTE_CMD_8: return 8;
    case IR_REMOTE_CMD_9: return 9;
    default: return -1;
  }
}

static StepperAxisId InfraredRemote_ToStepperAxis(InfraredControlAxis axis)
{
  return (axis == IR_CONTROL_AXIS_X) ? STEPPER_AXIS_X : STEPPER_AXIS_Z;
}

static void InfraredRemote_ApplyMotion(InfraredControlAxis axis, InfraredMotionState state)
{
  StepperAxisId stepper_axis = InfraredRemote_ToStepperAxis(axis);

  if (state == IR_MOTION_STOP)
  {
    StepperAxis_SetEnabled(stepper_axis, 0U);
    g_motion_state[axis] = IR_MOTION_STOP;
    return;
  }

  /* 启动前先在无脉冲状态写入方向，避免 DIR 与首个脉冲距离过短。 */
  StepperAxis_SetEnabled(stepper_axis, 0U);
  StepperAxis_SetDirectionReverse(stepper_axis, (state == IR_MOTION_REVERSE) ? 1U : 0U);
  StepperAxis_SetEnabled(stepper_axis, 1U);
  g_motion_state[axis] = StepperAxis_IsEnabled(stepper_axis) ? state : IR_MOTION_STOP;
}

static void InfraredRemote_SelectAxis(InfraredControlAxis axis)
{
  if (axis == g_selected_axis) return;

  /* 手动调试只允许单轴运动，切轴时先关闭原轴。 */
  InfraredRemote_ApplyMotion(g_selected_axis, IR_MOTION_STOP);
  g_direction_change_pending[g_selected_axis] = 0U;
  g_selected_axis = axis;
}

static void InfraredRemote_SelectTarget(InfraredControlTarget target)
{
  if (target == g_selected_target) return;

  /* 离开步进控制对象前先停轴，避免切到舵机后步进电机继续运动。 */
  if ((g_selected_target == IR_CONTROL_TARGET_X) ||
      (g_selected_target == IR_CONTROL_TARGET_Z))
  {
    InfraredRemote_ApplyMotion(g_selected_axis, IR_MOTION_STOP);
    g_direction_change_pending[g_selected_axis] = 0U;
  }
  else if (g_selected_target == IR_CONTROL_TARGET_CHASSIS)
  {
    ChassisMotion_StopManual();
  }

  g_selected_target = target;
  if (target == IR_CONTROL_TARGET_X)
    InfraredRemote_SelectAxis(IR_CONTROL_AXIS_X);
  else if (target == IR_CONTROL_TARGET_Z)
    InfraredRemote_SelectAxis(IR_CONTROL_AXIS_Z);
}

static void InfraredRemote_StopSelectedTarget(void)
{
  if (g_selected_target == IR_CONTROL_TARGET_CHASSIS)
  {
    ChassisMotion_StopManual();
  }
  else if ((g_selected_target == IR_CONTROL_TARGET_X) ||
           (g_selected_target == IR_CONTROL_TARGET_Z))
  {
    InfraredRemote_ApplyMotion(g_selected_axis, IR_MOTION_STOP);
    g_direction_change_pending[g_selected_axis] = 0U;
  }
}

void InfraredRemote_SyncToCurrentPage(void)
{
  if ((UiManager_GetPage() != UI_PAGE_DROP_DEMO) &&
      (DropDemo_GetState() != DROP_DEMO_IDLE)) DropDemo_Abort();
  if ((UiManager_GetPage() != UI_PAGE_BEAN_PICKUP_DEMO) &&
      (BeanPickupDemo_GetState() != BEAN_PICKUP_DEMO_IDLE))
    BeanPickupDemo_Abort();
  if ((UiManager_GetPage() != UI_PAGE_BEAN_SEQUENCE_DEMO) &&
      (BeanSequenceDemo_GetState() != BEAN_SEQUENCE_DEMO_IDLE))
    BeanSequenceDemo_Abort();
  if ((UiManager_GetPage() != UI_PAGE_XY_WAYPOINT_DEMO) &&
      (XyWaypointDemo_GetState() != XY_WAYPOINT_DEMO_IDLE))
    XyWaypointDemo_Abort();
  if ((UiManager_GetPage() != UI_PAGE_INITIALIZATION_DEBUG) &&
      (InitializationDebug_GetState() != INITIALIZATION_DEBUG_IDLE))
    InitializationDebug_Abort();
  if ((UiManager_GetPage() != UI_PAGE_ODOMETRY_CALIBRATION) &&
      OdometryCalibration_IsRunning()) OdometryCalibration_Abort();
  if ((UiManager_GetPage() != UI_PAGE_VISION) &&
      (UiManager_GetPage() != UI_PAGE_COMPETITION) &&
      (VisionRouteDemo_GetState() != VISION_ROUTE_DEMO_IDLE))
    VisionRouteDemo_Abort();
  switch (UiManager_GetPage())
  {
    case UI_PAGE_MOTOR_SPEED:
      InfraredRemote_SelectTarget(IR_CONTROL_TARGET_CHASSIS);
      break;

    case UI_PAGE_ODOMETRY_CALIBRATION:
      InfraredRemote_StopSelectedTarget();
      InfraredRemote_SelectTarget(IR_CONTROL_TARGET_CHASSIS);
      break;

    case UI_PAGE_COMPETITION:
      InfraredRemote_StopSelectedTarget();
      break;

    case UI_PAGE_STEPPER:
      if (StepperAxis_IsPulseMoveActive(InfraredRemote_ToStepperAxis(g_selected_axis)))
        InfraredRemote_ApplyMotion(g_selected_axis, IR_MOTION_STOP);
      InfraredRemote_SelectTarget(
          (g_selected_axis == IR_CONTROL_AXIS_X) ?
          IR_CONTROL_TARGET_X : IR_CONTROL_TARGET_Z);
      break;

    case UI_PAGE_STEPPER_PULSE:
      if (StepperAxis_IsEnabled(InfraredRemote_ToStepperAxis(g_selected_axis)) &&
          !StepperAxis_IsPulseMoveActive(InfraredRemote_ToStepperAxis(g_selected_axis)))
        InfraredRemote_ApplyMotion(g_selected_axis, IR_MOTION_STOP);
      InfraredRemote_SelectTarget(
          (g_selected_axis == IR_CONTROL_AXIS_X) ?
          IR_CONTROL_TARGET_X : IR_CONTROL_TARGET_Z);
      break;

    case UI_PAGE_BOX_CALIBRATION:
      InfraredRemote_StopSelectedTarget();
      InfraredRemote_SelectAxis(IR_CONTROL_AXIS_X);
      g_selected_target = IR_CONTROL_TARGET_X;
      break;

    case UI_PAGE_Z_CALIBRATION:
      InfraredRemote_StopSelectedTarget();
      InfraredRemote_SelectAxis(IR_CONTROL_AXIS_Z);
      g_selected_target = IR_CONTROL_TARGET_Z;
      break;

    case UI_PAGE_INITIALIZATION_DEBUG:
      InfraredRemote_StopSelectedTarget();
      break;

    case UI_PAGE_DROP_DEMO:
      InfraredRemote_StopSelectedTarget();
      break;

    case UI_PAGE_BEAN_PICKUP_DEMO:
      InfraredRemote_StopSelectedTarget();
      break;

    case UI_PAGE_BEAN_SEQUENCE_DEMO:
      InfraredRemote_StopSelectedTarget();
      break;

    case UI_PAGE_XY_WAYPOINT_DEMO:
      InfraredRemote_StopSelectedTarget();
      break;

    case UI_PAGE_SERVO:
      if ((g_selected_target != IR_CONTROL_TARGET_SERVO_1) &&
          (g_selected_target != IR_CONTROL_TARGET_SERVO_2))
      {
        InfraredRemote_SelectTarget(IR_CONTROL_TARGET_SERVO_1);
      }
      break;

    case UI_PAGE_OVERVIEW:
    case UI_PAGE_VISION:
    case UI_PAGE_SYSTEM:
    default:
      InfraredRemote_StopSelectedTarget();
      break;
  }
}

static void InfraredRemote_SelectDirection(InfraredControlAxis axis,
                                           InfraredMotionState direction)
{
  InfraredRemote_SelectAxis(axis);
  if ((g_motion_state[axis] != IR_MOTION_STOP) &&
      (g_motion_state[axis] != direction))
  {
    InfraredRemote_ApplyMotion(axis, IR_MOTION_STOP);
    g_direction_change_pending[axis] = 1U;
    g_direction_resume_tick[axis] = HAL_GetTick() + IR_X_DIRECTION_CHANGE_DELAY_MS;
  }
  else if (g_direction_change_pending[axis] != 0U)
  {
    g_direction_resume_tick[axis] = HAL_GetTick() + IR_X_DIRECTION_CHANGE_DELAY_MS;
  }
  g_selected_direction[axis] = direction;
}

static uint8_t InfraredRemote_InRange(uint16_t value, uint16_t minimum, uint16_t maximum)
{
  return (value >= minimum) && (value <= maximum);
}

static void InfraredRemote_ResetFrame(void)
{
  g_frame = 0U;
  g_bit_index = 0U;
  g_decode_state = IR_WAIT_LEADER_MARK;
}

HAL_StatusTypeDef InfraredRemote_Init(TIM_HandleTypeDef *htim)
{
  if ((htim == NULL) || (htim->Instance != TIM4))
  {
    return HAL_ERROR;
  }

  g_ir_timer = htim;
  g_last_capture = 0U;
  g_frame = 0U;
  g_bit_index = 0U;
  g_capture_started = 0U;
  g_last_edge_tick = HAL_GetTick();
  g_decode_state = IR_WAIT_LEADER_MARK;
  g_pending_command = 0U;
  g_command_ready = 0U;
  g_last_command = 0U;
  g_pulse_input = 0U;
  g_selected_axis = IR_CONTROL_AXIS_X;
  g_selected_target = IR_CONTROL_TARGET_CHASSIS;
  for (uint8_t axis = 0U; axis < IR_CONTROL_AXIS_COUNT; ++axis)
  {
    g_motion_state[axis] = IR_MOTION_STOP;
    g_selected_direction[axis] = IR_MOTION_FORWARD;
    g_direction_change_pending[axis] = 0U;
    g_direction_resume_tick[axis] = 0U;
    InfraredRemote_ApplyMotion((InfraredControlAxis)axis, IR_MOTION_STOP);
  }

  return HAL_TIM_IC_Start_IT(g_ir_timer, TIM_CHANNEL_4);
}

void InfraredRemote_HandleCapture(TIM_HandleTypeDef *htim)
{
  uint16_t capture;
  uint16_t interval_us;
  uint8_t address;
  uint8_t address_inverse;
  uint8_t command;
  uint8_t command_inverse;

  if ((htim != g_ir_timer) || (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_4))
  {
    return;
  }

  capture = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
  g_last_edge_tick = HAL_GetTick();
  if (g_capture_started == 0U)
  {
    g_last_capture = capture;
    g_capture_started = 1U;
    /* 翻转捕获极性，使下一次捕获在相反的边沿触发。 */
    g_ir_timer->Instance->CCER ^= TIM_CCER_CC4P;
    return;
  }

  /* TIM4 与摄像头舵机共用20ms周期，按ARR+1显式处理计数器回绕。 */
  if (capture >= g_last_capture)
    interval_us = capture - g_last_capture;
  else
    interval_us = (uint16_t)((g_ir_timer->Instance->ARR + 1U - g_last_capture) + capture);
  g_last_capture = capture;

  switch (g_decode_state)
  {
    case IR_WAIT_LEADER_MARK:
      if (InfraredRemote_InRange(interval_us, IR_LEADER_MARK_MIN_US, IR_LEADER_MARK_MAX_US))
      {
        g_decode_state = IR_WAIT_LEADER_SPACE;
      }
      break;

    case IR_WAIT_LEADER_SPACE:
      if (InfraredRemote_InRange(interval_us, IR_LEADER_SPACE_MIN_US, IR_LEADER_SPACE_MAX_US))
      {
        InfraredRemote_ResetFrame();
        g_decode_state = IR_WAIT_BIT_MARK;
      }
      else
      {
        g_decode_state = IR_WAIT_LEADER_MARK;
      }
      break;

    case IR_WAIT_BIT_MARK:
      if (InfraredRemote_InRange(interval_us, IR_BIT_MARK_MIN_US, IR_BIT_MARK_MAX_US))
      {
        g_decode_state = IR_WAIT_BIT_SPACE;
      }
      else
      {
        g_decode_state = IR_WAIT_LEADER_MARK;
      }
      break;

    case IR_WAIT_BIT_SPACE:
      if (InfraredRemote_InRange(interval_us, IR_BIT_ZERO_SPACE_MIN_US, IR_BIT_ZERO_SPACE_MAX_US))
      {
        /* NEC 按低位在前发送，位序可直接写入对应索引。 */
      }
      else if (InfraredRemote_InRange(interval_us, IR_BIT_ONE_SPACE_MIN_US, IR_BIT_ONE_SPACE_MAX_US))
      {
        g_frame |= (1UL << g_bit_index);
      }
      else
      {
        g_decode_state = IR_WAIT_LEADER_MARK;
        break;
      }

      g_bit_index++;
      g_decode_state = IR_WAIT_BIT_MARK;
      if (g_bit_index == IR_FRAME_BITS)
      {
        address = (uint8_t)g_frame;
        address_inverse = (uint8_t)(g_frame >> 8U);
        command = (uint8_t)(g_frame >> 16U);
        command_inverse = (uint8_t)(g_frame >> 24U);
        /* 同时校验地址、命令反码，并过滤未定义按键，避免上电噪声误鸣。 */
        if (((uint8_t)(address ^ address_inverse) == 0xFFU) &&
            ((uint8_t)(command ^ command_inverse) == 0xFFU) &&
            InfraredRemote_IsKnownCommand(command))
        {
          g_pending_command = command;
          g_command_ready = 1U;
        }
        InfraredRemote_ResetFrame();
        g_decode_state = IR_WAIT_LEADER_MARK;
      }
      break;

    default:
      g_decode_state = IR_WAIT_LEADER_MARK;
      break;
  }

  /* 翻转捕获极性，使下一次捕获在相反的边沿触发。
   * TIM4（通用定时器）无硬件 BOTHEDGE 支持，需软件交替。 */
  g_ir_timer->Instance->CCER ^= TIM_CCER_CC4P;
}

void InfraredRemote_Process(void)
{
  uint8_t command;
  uint8_t safety_fault;
  uint32_t primask;
  UiPage page;

  /* 丢边沿会破坏软件交替捕获的极性；空闲超时后强制回到下降沿起始态。 */
  if ((g_capture_started != 0U) &&
      ((uint32_t)(HAL_GetTick() - g_last_edge_tick) > IR_FRAME_TIMEOUT_MS))
  {
    uint32_t reset_primask = __get_PRIMASK();
    __disable_irq();
    g_capture_started = 0U;
    InfraredRemote_ResetFrame();
    __HAL_TIM_SET_CAPTUREPOLARITY(g_ir_timer, TIM_CHANNEL_4,
                                  TIM_INPUTCHANNELPOLARITY_FALLING);
    __set_PRIMASK(reset_primask);
  }

  /* PB11由X/Z轴共用：记录触发方向，停机后只允许反向脱离。 */
  StepperAxis_ProcessPhotoInterlock();
  if (PhotoSensor_GetState(PHOTO_SENSOR_SHARED_XZ) != 0U)
  {
    for (uint8_t axis = 0U; axis < IR_CONTROL_AXIS_COUNT; ++axis)
    {
      if (!StepperAxis_IsEnabled(InfraredRemote_ToStepperAxis((InfraredControlAxis)axis)))
      {
        g_motion_state[axis] = IR_MOTION_STOP;
        g_direction_change_pending[axis] = 0U;
      }
    }
  }

  /* 急停或限位故障优先级最高，禁止待执行命令重新启动 X 轴。 */
  safety_fault = ((SafetyManager_GetFlags() &
                  (SAFETY_FAULT_ESTOP | SAFETY_FAULT_LIMIT)) != 0U) ? 1U : 0U;
  if (safety_fault != 0U)
  {
    for (uint8_t axis = 0U; axis < IR_CONTROL_AXIS_COUNT; ++axis)
    {
      InfraredRemote_ApplyMotion((InfraredControlAxis)axis, IR_MOTION_STOP);
      g_direction_change_pending[axis] = 0U;
    }
  }

  /* 换向采用“先停机、等待死区、再恢复”的非阻塞状态机。 */
  if ((safety_fault == 0U) &&
      (g_direction_change_pending[g_selected_axis] != 0U) &&
      ((int32_t)(HAL_GetTick() - g_direction_resume_tick[g_selected_axis]) >= 0))
  {
    InfraredRemote_ApplyMotion(g_selected_axis, g_selected_direction[g_selected_axis]);
    g_direction_change_pending[g_selected_axis] = 0U;
  }

  if (g_command_ready == 0U)
  {
    return;
  }

  /* 仅用极短临界区取走 ISR 给出的单条命令。 */
  primask = __get_PRIMASK();
  __disable_irq();
  command = g_pending_command;
  g_command_ready = 0U;
  __set_PRIMASK(primask);

  g_last_command = command;
  Buzzer_Beep(IR_KEY_BEEP_DURATION_MS);

  /* 页面浏览不产生运动，即使故障锁定也允许查看诊断信息。 */
  if (command == IR_REMOTE_CMD_VOL_MINUS)
  {
    if (UiManager_GetPage() == UI_PAGE_DROP_DEMO) DropDemo_Abort();
    if (UiManager_GetPage() == UI_PAGE_BEAN_PICKUP_DEMO) BeanPickupDemo_Abort();
    if (UiManager_GetPage() == UI_PAGE_BEAN_SEQUENCE_DEMO) BeanSequenceDemo_Abort();
    if (UiManager_GetPage() == UI_PAGE_XY_WAYPOINT_DEMO) XyWaypointDemo_Abort();
    if (UiManager_GetPage() == UI_PAGE_INITIALIZATION_DEBUG)
      InitializationDebug_Abort();
    if (UiManager_GetPage() == UI_PAGE_ODOMETRY_CALIBRATION)
      OdometryCalibration_Abort();
    if (UiManager_GetPage() == UI_PAGE_VISION) VisionRouteDemo_Abort();
    if ((UiManager_GetPage() == UI_PAGE_COMPETITION) &&
        (RobotController_GetState() >= ROBOT_STATE_SELF_CHECK) &&
        (RobotController_GetState() <= ROBOT_STATE_RETURN_FINISH))
      RobotController_RequestAbort();
    UiManager_PreviousPage();
    InfraredRemote_SyncToCurrentPage();
    return;
  }
  if (command == IR_REMOTE_CMD_VOL_PLUS)
  {
    if (UiManager_GetPage() == UI_PAGE_DROP_DEMO) DropDemo_Abort();
    if (UiManager_GetPage() == UI_PAGE_BEAN_PICKUP_DEMO) BeanPickupDemo_Abort();
    if (UiManager_GetPage() == UI_PAGE_BEAN_SEQUENCE_DEMO) BeanSequenceDemo_Abort();
    if (UiManager_GetPage() == UI_PAGE_XY_WAYPOINT_DEMO) XyWaypointDemo_Abort();
    if (UiManager_GetPage() == UI_PAGE_INITIALIZATION_DEBUG)
      InitializationDebug_Abort();
    if (UiManager_GetPage() == UI_PAGE_ODOMETRY_CALIBRATION)
      OdometryCalibration_Abort();
    if (UiManager_GetPage() == UI_PAGE_VISION) VisionRouteDemo_Abort();
    if ((UiManager_GetPage() == UI_PAGE_COMPETITION) &&
        (RobotController_GetState() >= ROBOT_STATE_SELF_CHECK) &&
        (RobotController_GetState() <= ROBOT_STATE_RETURN_FINISH))
      RobotController_RequestAbort();
    UiManager_NextPage();
    InfraredRemote_SyncToCurrentPage();
    return;
  }

  page = UiManager_GetPage();

  /* 故障状态只确认收到按键，不执行任何电机动作。 */
  if (safety_fault != 0U) return;

  if (page == UI_PAGE_DROP_DEMO)
  {
    if (command == IR_REMOTE_CMD_POWER) DropDemo_ToggleRunning();
    else if (command == IR_REMOTE_CMD_0) DropDemo_Abort();
    return;
  }

  if (page == UI_PAGE_BEAN_PICKUP_DEMO)
  {
    if (!BeanPickupDemo_IsRunning() && (command == IR_REMOTE_CMD_1))
      BeanPickupDemo_SelectPosition(0U);
    else if (!BeanPickupDemo_IsRunning() && (command == IR_REMOTE_CMD_2))
      BeanPickupDemo_SelectPosition(1U);
    else if (!BeanPickupDemo_IsRunning() && (command == IR_REMOTE_CMD_3))
      BeanPickupDemo_SelectPosition(2U);
    else if (command == IR_REMOTE_CMD_POWER)
      BeanPickupDemo_HandlePower();
    else if (command == IR_REMOTE_CMD_0)
      BeanPickupDemo_Abort();
    return;
  }

  if (page == UI_PAGE_BEAN_SEQUENCE_DEMO)
  {
    if (command == IR_REMOTE_CMD_POWER) BeanSequenceDemo_Start();
    else if (command == IR_REMOTE_CMD_0) BeanSequenceDemo_Abort();
    return;
  }

  if (page == UI_PAGE_XY_WAYPOINT_DEMO)
  {
    XyWaypointId target = XY_WAYPOINT_COUNT;
    if (command == IR_REMOTE_CMD_1) target = XY_WAYPOINT_NUMBER_1;
    else if (command == IR_REMOTE_CMD_2) target = XY_WAYPOINT_NUMBER_2;
    else if (command == IR_REMOTE_CMD_3) target = XY_WAYPOINT_NUMBER_3;
    else if (command == IR_REMOTE_CMD_4) target = XY_WAYPOINT_NUMBER_4;
    else if (command == IR_REMOTE_CMD_5) target = XY_WAYPOINT_NUMBER_5;
    else if (command == IR_REMOTE_CMD_7) target = XY_WAYPOINT_A;
    else if (command == IR_REMOTE_CMD_8) target = XY_WAYPOINT_B;
    else if (command == IR_REMOTE_CMD_9) target = XY_WAYPOINT_C;

    if (target < XY_WAYPOINT_COUNT) XyWaypointDemo_Select(target);
    else if (command == IR_REMOTE_CMD_POWER) XyWaypointDemo_HandlePower();
    else if (command == IR_REMOTE_CMD_0) XyWaypointDemo_Abort();
    return;
  }

  if (page == UI_PAGE_INITIALIZATION_DEBUG)
  {
    if (command == IR_REMOTE_CMD_POWER) InitializationDebug_ToggleRunning();
    else if (command == IR_REMOTE_CMD_0) InitializationDebug_Abort();
    return;
  }

  if (page == UI_PAGE_ODOMETRY_CALIBRATION)
  {
    int8_t digit = InfraredRemote_DigitValue(command);
    if (digit >= 0)
      OdometryCalibration_AppendDigit((uint8_t)digit);
    else if (command == IR_REMOTE_CMD_LEFT)
      OdometryCalibration_SelectField(ODOMETRY_FIELD_DISTANCE);
    else if (command == IR_REMOTE_CMD_RIGHT)
      OdometryCalibration_SelectField(ODOMETRY_FIELD_SPEED);
    else if (command == IR_REMOTE_CMD_UP)
      OdometryCalibration_SetDirection(0U);
    else if (command == IR_REMOTE_CMD_DOWN)
      OdometryCalibration_SetDirection(1U);
    else if (command == IR_REMOTE_CMD_POWER)
      OdometryCalibration_ToggleRunning();
    return;
  }

  /* 视觉页待机时：1发'N'选数字，2发'B'选豆子。
   * 自动路线运行中禁止手动切换，避免破坏当前采集窗口。 */
  if (page == UI_PAGE_VISION)
  {
    if (command == IR_REMOTE_CMD_POWER)
      VisionRouteDemo_ToggleRunning();
    else if (command == IR_REMOTE_CMD_0)
      VisionRouteDemo_Abort();
    else if (!VisionRouteDemo_IsRunning() && (command == IR_REMOTE_CMD_1))
    {
      (void)K230Link_SelectTask(K230_TASK_NUMBER);
    }
    else if (!VisionRouteDemo_IsRunning() && (command == IR_REMOTE_CMD_2))
    {
      (void)K230Link_SelectTask(K230_TASK_BEAN);
    }
    return;
  }

  if (page == UI_PAGE_COMPETITION)
  {
    if (command == IR_REMOTE_CMD_POWER) RobotController_RequestStart();
    else if (command == IR_REMOTE_CMD_0) RobotController_RequestAbort();
    return;
  }

  /* 箱位标定页使用有限脉冲点动，所有命令均在本页内消费。 */
  if (page == UI_PAGE_BOX_CALIBRATION)
  {
    int8_t digit = InfraredRemote_DigitValue(command);
    if ((digit >= 1) && (digit <= 5))
      BoxCalibration_SelectSlot((uint8_t)(digit - 1));
    else if (command == IR_REMOTE_CMD_LEFT)
      (void)BoxCalibration_Jog(1U);
    else if (command == IR_REMOTE_CMD_RIGHT)
      (void)BoxCalibration_Jog(0U);
    else if (command == IR_REMOTE_CMD_UP)
      BoxCalibration_AdjustStep(1);
    else if (command == IR_REMOTE_CMD_DOWN)
      BoxCalibration_AdjustStep(-1);
    else if (command == IR_REMOTE_CMD_POWER)
      (void)BoxCalibration_SaveCurrent();
    else if (command == IR_REMOTE_CMD_0)
      (void)BoxCalibration_SetXZero();
    return;
  }

  /* Z高度标定页：三档夹取高度和一个放豆高度。 */
  if (page == UI_PAGE_Z_CALIBRATION)
  {
    int8_t digit = InfraredRemote_DigitValue(command);
    if ((digit >= 1) && (digit <= 4))
      ZCalibration_SelectTarget((uint8_t)(digit - 1));
    else if (command == IR_REMOTE_CMD_UP)
      (void)ZCalibration_Jog(1U);
    else if (command == IR_REMOTE_CMD_DOWN)
      (void)ZCalibration_Jog(0U);
    else if (command == IR_REMOTE_CMD_LEFT)
      ZCalibration_AdjustStep(-1);
    else if (command == IR_REMOTE_CMD_RIGHT)
      ZCalibration_AdjustStep(1);
    else if (command == IR_REMOTE_CMD_POWER)
      (void)ZCalibration_SaveCurrent();
    else if (command == IR_REMOTE_CMD_0)
      (void)ZCalibration_SetBottomReference();
    return;
  }

  /* 定量页的数字键逐位输入，六位上限可确保OLED整行显示。 */
  if (page == UI_PAGE_STEPPER_PULSE)
  {
    int8_t digit = InfraredRemote_DigitValue(command);
    if (digit >= 0)
    {
      if (!StepperAxis_IsPulseMoveActive(InfraredRemote_ToStepperAxis(g_selected_axis)) &&
          (g_pulse_input <= 99999U))
      {
        g_pulse_input = g_pulse_input * 10U + (uint32_t)digit;
      }
      return;
    }
  }

  /* 手动页停止时按0清零当前轴的标定脉冲；其他页面仍为全局停止。 */
  if ((page == UI_PAGE_STEPPER) && (command == IR_REMOTE_CMD_0))
  {
    (void)StepperAxis_ResetPositionPulses(InfraredRemote_ToStepperAxis(g_selected_axis));
    return;
  }
  if (command == IR_REMOTE_CMD_0)
  {
    ChassisMotion_StopManual();
    for (uint8_t axis = 0U; axis < IR_CONTROL_AXIS_COUNT; ++axis)
    {
      InfraredRemote_ApplyMotion((InfraredControlAxis)axis, IR_MOTION_STOP);
      g_direction_change_pending[axis] = 0U;
    }
    UiManager_SetPage(UI_PAGE_OVERVIEW);
    return;
  }

  InfraredRemote_SyncToCurrentPage();

  /* 数字键只在当前机构页面内选择子对象或调整参数。 */
  if (page == UI_PAGE_STEPPER)
  {
    if (command == IR_REMOTE_CMD_2)
    {
      InfraredRemote_SelectTarget(IR_CONTROL_TARGET_X);
      return;
    }
    if (command == IR_REMOTE_CMD_3)
    {
      InfraredRemote_SelectTarget(IR_CONTROL_TARGET_Z);
      return;
    }
    if (command == IR_REMOTE_CMD_RIGHT)
    {
      InfraredRemote_SelectTarget(
          (g_selected_axis == IR_CONTROL_AXIS_X) ?
          IR_CONTROL_TARGET_Z : IR_CONTROL_TARGET_X);
      return;
    }
  }
  else if (page == UI_PAGE_STEPPER_PULSE)
  {
    StepperAxisId axis = InfraredRemote_ToStepperAxis(g_selected_axis);
    if (command == IR_REMOTE_CMD_RIGHT)
    {
      if (!StepperAxis_IsPulseMoveActive(axis))
      {
        InfraredRemote_SelectTarget(
            (g_selected_axis == IR_CONTROL_AXIS_X) ?
            IR_CONTROL_TARGET_Z : IR_CONTROL_TARGET_X);
      }
      return;
    }
    if (command == IR_REMOTE_CMD_LEFT)
    {
      if (!StepperAxis_IsPulseMoveActive(axis)) g_pulse_input /= 10U;
      return;
    }
  }
  else if (page == UI_PAGE_SERVO)
  {
    if (command == IR_REMOTE_CMD_4)
    {
      InfraredRemote_SelectTarget(IR_CONTROL_TARGET_SERVO_1);
      return;
    }
    if (command == IR_REMOTE_CMD_5)
    {
      InfraredRemote_SelectTarget(IR_CONTROL_TARGET_SERVO_2);
      return;
    }
  }
  else if (page == UI_PAGE_MOTOR_SPEED)
  {
    if (command == IR_REMOTE_CMD_4)
    {
      ChassisMotion_AdjustAlignTimeout(-100);
      return;
    }
    if (command == IR_REMOTE_CMD_5)
    {
      ChassisMotion_AdjustAlignTimeout(100);
      return;
    }
    if (command == IR_REMOTE_CMD_6)
    {
      (void)ChassisMotion_ToggleClosedLoop();
      return;
    }
    if (command == IR_REMOTE_CMD_7)
    {
      ChassisMotion_AdjustTargetRpm(-5);
      return;
    }
    if (command == IR_REMOTE_CMD_8)
    {
      ChassisMotion_AdjustTargetRpm(5);
      return;
    }
  }

  /* 总览和系统页只用于观察。 */
  if ((page == UI_PAGE_OVERVIEW) || (page == UI_PAGE_VISION) ||
      (page == UI_PAGE_COMPETITION) ||
      (page == UI_PAGE_SYSTEM)) return;

  if (command == IR_REMOTE_CMD_UP)
  {
    if (page == UI_PAGE_MOTOR_SPEED)
      (void)ChassisMotion_SetDirection(0U);
    else if ((page == UI_PAGE_STEPPER) || (page == UI_PAGE_STEPPER_PULSE))
    {
      if ((page == UI_PAGE_STEPPER) ||
          !StepperAxis_IsPulseMoveActive(InfraredRemote_ToStepperAxis(g_selected_axis)))
      {
        InfraredRemote_SelectDirection(g_selected_axis, IR_MOTION_FORWARD);
      }
    }
    else
      ServoControl_AdjustAngle(
          (g_selected_target == IR_CONTROL_TARGET_SERVO_2) ? 1U : 0U, 10);
  }
  else if (command == IR_REMOTE_CMD_DOWN)
  {
    if (page == UI_PAGE_MOTOR_SPEED)
      (void)ChassisMotion_SetDirection(1U);
    else if ((page == UI_PAGE_STEPPER) || (page == UI_PAGE_STEPPER_PULSE))
    {
      if ((page == UI_PAGE_STEPPER) ||
          !StepperAxis_IsPulseMoveActive(InfraredRemote_ToStepperAxis(g_selected_axis)))
      {
        InfraredRemote_SelectDirection(g_selected_axis, IR_MOTION_REVERSE);
      }
    }
    else
      ServoControl_AdjustAngle(
          (g_selected_target == IR_CONTROL_TARGET_SERVO_2) ? 1U : 0U, -10);
  }
  else if (command == IR_REMOTE_CMD_POWER)
  {
    if (page == UI_PAGE_MOTOR_SPEED)
    {
      ChassisMotion_ToggleRunning();
    }
    else if (page == UI_PAGE_SERVO)
    {
      ServoControl_ResetToInitial(
          (g_selected_target == IR_CONTROL_TARGET_SERVO_2) ? 1U : 0U);
    }
    else if (page == UI_PAGE_STEPPER_PULSE)
    {
      StepperAxisId axis = InfraredRemote_ToStepperAxis(g_selected_axis);
      if (StepperAxis_IsPulseMoveActive(axis))
      {
        StepperAxis_SetEnabled(axis, 0U);
      }
      else if (g_pulse_input != 0U)
      {
        (void)StepperAxis_MovePulses(
            axis, g_pulse_input,
            (g_selected_direction[g_selected_axis] == IR_MOTION_REVERSE) ? 1U : 0U);
      }
    }
    else if (g_direction_change_pending[g_selected_axis] != 0U)
    {
      g_direction_change_pending[g_selected_axis] = 0U;
      InfraredRemote_ApplyMotion(g_selected_axis, IR_MOTION_STOP);
    }
    else
    {
      InfraredRemote_ApplyMotion(g_selected_axis,
          (g_motion_state[g_selected_axis] == IR_MOTION_STOP) ?
          g_selected_direction[g_selected_axis] : IR_MOTION_STOP);
    }
  }
}

uint8_t InfraredRemote_GetLastCommand(void)
{
  return g_last_command;
}

InfraredControlAxis InfraredRemote_GetSelectedAxis(void)
{
  return g_selected_axis;
}

InfraredMotionState InfraredRemote_GetSelectedMotionState(void)
{
  return g_motion_state[g_selected_axis];
}

uint8_t InfraredRemote_GetSelectedDirectionReverse(void)
{
  return (g_selected_direction[g_selected_axis] == IR_MOTION_REVERSE) ? 1U : 0U;
}

uint8_t InfraredRemote_IsDirectionChangePending(void)
{
  return g_direction_change_pending[g_selected_axis];
}

InfraredControlTarget InfraredRemote_GetSelectedTarget(void)
{
  return g_selected_target;
}

uint8_t InfraredRemote_GetSelectedServoIndex(void)
{
  return (g_selected_target == IR_CONTROL_TARGET_SERVO_2) ? 1U : 0U;
}

uint32_t InfraredRemote_GetPulseInput(void)
{
  return g_pulse_input;
}
