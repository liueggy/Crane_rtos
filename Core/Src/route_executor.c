#include "route_executor.h"

#include "app_config.h"
#include "chassis_motion.h"
#include "stepper_axis.h"
#include "world_map.h"

static RouteExecutorState g_state;
static RobotRouteId g_route_id;
static uint8_t g_segment_index;
static uint8_t g_center_cross_active;
static int32_t g_center_cross_target_x;

static uint8_t StartCenterCross(void)
{
  int32_t current_x = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
  int32_t midpoint = (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U);
  int32_t target = (current_x < midpoint) ?
                   (int32_t)(STEPPER_X_TRAVEL_PULSES * 3U / 4U) :
                   (int32_t)(STEPPER_X_TRAVEL_PULSES / 4U);
  uint32_t pulses = (uint32_t)((target > current_x) ?
                              (target - current_x) : (current_x - target));

  if ((pulses == 0U) ||
      (StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) >
       WorldMap_GetTransportSafeZPulses())) return 0U;
  g_center_cross_target_x = target;
  g_center_cross_active = 1U;
  return StepperAxis_MovePulses(STEPPER_AXIS_X, pulses,
                               (target < current_x) ? 1U : 0U) == HAL_OK;
}

static uint8_t StartCurrentSegment(void)
{
  const RobotRoute *route = RobotRoutes_Get(g_route_id);
  if ((route == 0) || (g_segment_index >= route->count)) return 0U;
  const RobotRouteSegment *segment = &route->segments[g_segment_index];
  if (!segment->calibrated) return 0U;
  if (WorldMap_RequiresBlockedAlignment(segment->expected_station))
    return ChassisMotion_StartAlignedRouteSegment(
        segment->distance_mm, segment->turn_deg,
        segment->speed_rpm, segment->timeout_ms);
  return ChassisMotion_StartRouteSegment(segment->distance_mm, segment->turn_deg,
                                         segment->speed_rpm, segment->timeout_ms);
}

void RouteExecutor_Init(void)
{
  g_state = ROUTE_EXECUTOR_IDLE;
  g_route_id = ROBOT_ROUTE_SURVEY_START_TO_NUMBER;
  g_segment_index = 0U;
  g_center_cross_active = 0U;
  g_center_cross_target_x = 0;
}

uint8_t RouteExecutor_Start(RobotRouteId route_id)
{
  if ((g_state == ROUTE_EXECUTOR_RUNNING) || (route_id >= ROBOT_ROUTE_COUNT)) return 0U;
  if (!RobotRoutes_IsCalibrated(route_id) ||
      !RobotRoutes_HasRequiredCenterPass(route_id))
  {
    g_state = ROUTE_EXECUTOR_NEEDS_CALIBRATION;
    return 0U;
  }
  /* 夹爪未完全抬到运输高度时禁止底盘穿越障碍区。 */
  if (StepperAxis_IsEnabled(STEPPER_AXIS_Z) ||
      (StepperAxis_GetPositionPulses(STEPPER_AXIS_Z) >
       WorldMap_GetTransportSafeZPulses()))
  {
    g_state = ROUTE_EXECUTOR_FAULT;
    return 0U;
  }
  g_route_id = route_id;
  g_segment_index = 0U;
  g_center_cross_active = 0U;
  if (!StartCurrentSegment())
  {
    g_state = ROUTE_EXECUTOR_FAULT;
    return 0U;
  }
  g_state = ROUTE_EXECUTOR_RUNNING;
  return 1U;
}

void RouteExecutor_Update(void)
{
  const RobotRoute *route;
  if (g_state != ROUTE_EXECUTOR_RUNNING) return;
  if (g_center_cross_active)
  {
    if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X)) return;
    if (StepperAxis_GetPositionPulses(STEPPER_AXIS_X) != g_center_cross_target_x)
    {
      g_state = ROUTE_EXECUTOR_FAULT;
      return;
    }
    WorldMap_SetAxisPosition(g_center_cross_target_x, 1U,
                             StepperAxis_GetPositionPulses(STEPPER_AXIS_Z), 1U);
    g_center_cross_active = 0U;
    if (g_segment_index >= RobotRoutes_Get(g_route_id)->count)
      g_state = ROUTE_EXECUTOR_DONE;
    else if (!StartCurrentSegment())
      g_state = ROUTE_EXECUTOR_FAULT;
    return;
  }
  if (ChassisMotion_DidRouteSegmentFail())
  {
    WorldMap_InvalidateY();
    g_state = ROUTE_EXECUTOR_FAULT;
    return;
  }
  if (!ChassisMotion_IsRouteSegmentDone()) return;

  route = RobotRoutes_Get(g_route_id);
  if (route == 0)
  {
    g_state = ROUTE_EXECUTOR_FAULT;
    return;
  }
  WorldMap_SetKnownStation(route->segments[g_segment_index].expected_station);
  if ((route->policy == ROBOT_ROUTE_POLICY_REQUIRE_CENTER_PASS) &&
      (route->segments[g_segment_index].expected_station == WORLD_STATION_START))
  {
    ++g_segment_index;
    if (!StartCenterCross()) g_state = ROUTE_EXECUTOR_FAULT;
    return;
  }
  ++g_segment_index;
  if (g_segment_index >= route->count)
  {
    g_state = ROUTE_EXECUTOR_DONE;
    return;
  }
  if (!StartCurrentSegment()) g_state = ROUTE_EXECUTOR_FAULT;
}

void RouteExecutor_Abort(void)
{
  ChassisMotion_Stop();
  if (g_center_cross_active) StepperAxis_SetEnabled(STEPPER_AXIS_X, 0U);
  g_center_cross_active = 0U;
  g_state = ROUTE_EXECUTOR_IDLE;
}

RouteExecutorState RouteExecutor_GetState(void)
{
  return g_state;
}

RobotRouteId RouteExecutor_GetRoute(void)
{
  return g_route_id;
}

uint8_t RouteExecutor_GetSegmentIndex(void)
{
  return g_segment_index;
}
