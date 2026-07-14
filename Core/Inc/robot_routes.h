#ifndef ROBOT_ROUTES_H
#define ROBOT_ROUTES_H

#include <stdint.h>

typedef enum
{
  ROUTE_POINT_PICK_1 = 0,
  ROUTE_POINT_PICK_2,
  ROUTE_POINT_PICK_3,
  ROUTE_POINT_OBSTACLE_ENTRY,
  ROUTE_POINT_OBSTACLE_EXIT,
  ROUTE_POINT_DROP_1,
  ROUTE_POINT_DROP_2,
  ROUTE_POINT_DROP_3,
  ROUTE_POINT_FINISH
} RobotRoutePoint;

typedef struct
{
  RobotRoutePoint point;
  int16_t distance_mm;
  int16_t turn_deg;
  uint16_t speed_rpm;
  uint16_t timeout_ms;
} RobotRouteSegment;

typedef struct
{
  const RobotRouteSegment *segments;
  uint8_t count;
} RobotRoute;

const RobotRoute *RobotRoutes_GetPickRoute(uint8_t pick_position);
const RobotRoute *RobotRoutes_GetDropRoute(uint8_t target_box);
const RobotRoute *RobotRoutes_GetReturnRoute(void);

#endif
