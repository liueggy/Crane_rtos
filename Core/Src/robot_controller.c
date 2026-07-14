#include "robot_controller.h"

#include "app_state.h"
#include "chassis_motion.h"
#include "robot_routes.h"
#include "safety_manager.h"

static RobotState g_state;
static RobotTransportTask g_tasks[3];
static uint8_t g_task_index;
static uint8_t g_route_index;
static uint8_t g_start_requested;

static void EnterState(RobotState state)
{
  g_state = state;
  if (state == ROBOT_STATE_FAULT)
  {
    ChassisMotion_Stop();
    AppState_SetMode(APP_MODE_FAULT);
  }
  else if ((state != ROBOT_STATE_IDLE) && (state != ROBOT_STATE_FINISHED))
  {
    AppState_SetMode(APP_MODE_AUTO);
  }
}

void RobotController_Init(void)
{
  /* 目标映射由比赛规则固定；取货位置由视觉识别后写入。 */
  g_tasks[0] = (RobotTransportTask){1U, 0U, 1U}; /* 黄豆 -> 1号盒 */
  g_tasks[1] = (RobotTransportTask){2U, 0U, 2U}; /* 绿豆 -> 2号盒 */
  g_tasks[2] = (RobotTransportTask){3U, 0U, 3U}; /* 白芸豆 -> 3号盒 */
  g_task_index = 0U;
  g_route_index = 0U;
  g_start_requested = 0U;
  EnterState(ROBOT_STATE_IDLE);
}

void RobotController_RequestStart(void)
{
  if (!SafetyManager_IsEstopActive()) g_start_requested = 1U;
}

void RobotController_RequestAbort(void)
{
  EnterState(ROBOT_STATE_FAULT);
}

void RobotController_Update(void)
{
  const RobotRoute *route;
  if (SafetyManager_IsEstopActive())
  {
    EnterState(ROBOT_STATE_FAULT);
    return;
  }
  switch (g_state)
  {
    case ROBOT_STATE_IDLE:
      if (g_start_requested) EnterState(ROBOT_STATE_SELF_CHECK);
      break;
    case ROBOT_STATE_SELF_CHECK:
      /* 后续在此检查限位、视觉在线、底盘校准和料斗初始状态。 */
      EnterState(ROBOT_STATE_VISION_SCAN);
      break;
    case ROBOT_STATE_VISION_SCAN:
      /* K230完成三类豆子及数字盒的多帧确认后，再进入任务建表。 */
      break;
    case ROBOT_STATE_TASK_BUILD:
      g_task_index = 0U;
      EnterState(ROBOT_STATE_MOVE_TO_PICK);
      break;
    case ROBOT_STATE_MOVE_TO_PICK:
      route = RobotRoutes_GetPickRoute(g_tasks[g_task_index].pick_position);
      if ((route == 0) || (route->count == 0U))
      {
        EnterState(ROBOT_STATE_FAULT);
        break;
      }
      if (ChassisMotion_StartRouteSegment(route->segments[0].distance_mm,
                                           route->segments[0].turn_deg,
                                           route->segments[0].speed_rpm,
                                           route->segments[0].timeout_ms))
      {
        g_route_index = 0U;
        EnterState(ROBOT_STATE_PICK_ACTION);
      }
      break;
    case ROBOT_STATE_PICK_ACTION:
      /* 取料机构完成“对准、下降、收料、关闭”后由机构模块确认。 */
      break;
    case ROBOT_STATE_LIFT_SAFE:
      /* 抬升高度必须小于障碍物顶面；实际高度参数后续写入 AppConfig。 */
      break;
    case ROBOT_STATE_FOLLOW_ROUTE:
      route = RobotRoutes_GetDropRoute(g_tasks[g_task_index].target_box);
      if ((route == 0) || (g_route_index >= route->count))
      {
        EnterState(ROBOT_STATE_FAULT);
        break;
      }
      if (!ChassisMotion_IsRouteSegmentDone()) break;
      ++g_route_index;
      if (g_route_index >= route->count) EnterState(ROBOT_STATE_DROP_ACTION);
      else (void)ChassisMotion_StartRouteSegment(route->segments[g_route_index].distance_mm,
                                                   route->segments[g_route_index].turn_deg,
                                                   route->segments[g_route_index].speed_rpm,
                                                   route->segments[g_route_index].timeout_ms);
      break;
    case ROBOT_STATE_DROP_ACTION:
      /* 倾倒完成后需要确认爪具复位、料斗无残留，再处理下一任务。 */
      break;
    case ROBOT_STATE_RETURN_FINISH:
      /* 使用 RobotRoutes_GetReturnRoute() 回到放置区侧，随后锁定。 */
      break;
    case ROBOT_STATE_FINISHED:
    case ROBOT_STATE_FAULT:
    default:
      break;
  }
}

RobotState RobotController_GetState(void)
{
  return g_state;
}

const RobotTransportTask *RobotController_GetActiveTask(void)
{
  return (g_task_index < 3U) ? &g_tasks[g_task_index] : 0;
}
