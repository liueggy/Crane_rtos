#include "robot_routes.h"

/* 坐标与距离待实机标定；状态机只依赖路线接口，不依赖具体数值。 */
static const RobotRouteSegment k_pick_routes[3][1] = {
  {{ROUTE_POINT_PICK_1, 0, 0, 35U, 8000U}},
  {{ROUTE_POINT_PICK_2, 0, 0, 35U, 8000U}},
  {{ROUTE_POINT_PICK_3, 0, 0, 35U, 8000U}},
};
static const RobotRouteSegment k_drop_routes[3][3] = {
  {{ROUTE_POINT_OBSTACLE_ENTRY, 0, 0, 35U, 8000U}, {ROUTE_POINT_OBSTACLE_EXIT, 0, 0, 35U, 8000U}, {ROUTE_POINT_DROP_1, 0, 0, 25U, 8000U}},
  {{ROUTE_POINT_OBSTACLE_ENTRY, 0, 0, 35U, 8000U}, {ROUTE_POINT_OBSTACLE_EXIT, 0, 0, 35U, 8000U}, {ROUTE_POINT_DROP_2, 0, 0, 25U, 8000U}},
  {{ROUTE_POINT_OBSTACLE_ENTRY, 0, 0, 35U, 8000U}, {ROUTE_POINT_OBSTACLE_EXIT, 0, 0, 35U, 8000U}, {ROUTE_POINT_DROP_3, 0, 0, 25U, 8000U}},
};
static const RobotRouteSegment k_return_route[] = {{ROUTE_POINT_FINISH, 0, 0, 35U, 8000U}};

const RobotRoute *RobotRoutes_GetPickRoute(uint8_t pick_position)
{
  static RobotRoute route;
  if ((pick_position < 1U) || (pick_position > 3U)) return 0;
  route.segments = k_pick_routes[pick_position - 1U];
  route.count = 1U;
  return &route;
}

const RobotRoute *RobotRoutes_GetDropRoute(uint8_t target_box)
{
  static RobotRoute route;
  if ((target_box < 1U) || (target_box > 3U)) return 0;
  route.segments = k_drop_routes[target_box - 1U];
  route.count = 3U;
  return &route;
}

const RobotRoute *RobotRoutes_GetReturnRoute(void)
{
  static const RobotRoute route = {k_return_route, 1U};
  return &route;
}
