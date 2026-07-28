#ifndef ROBOT_ROUTES_H
#define ROBOT_ROUTES_H

#include "world_map.h"

#include <stdint.h>

/* 当前联调阶段所有路线统一低速，后续完成逐段验证后再分别提速。 */
#define ROBOT_ROUTE_CRUISE_RPM  50U
#define ROBOT_ROUTE_APPROACH_RPM 50U

typedef enum
{
  ROBOT_ROUTE_SURVEY_START_TO_NUMBER = 0,
  ROBOT_ROUTE_SURVEY_NUMBER_TO_BEAN_DIRECT,
  ROBOT_ROUTE_TRANSPORT_BEAN_TO_NUMBER_CENTER,
  ROBOT_ROUTE_TRANSPORT_NUMBER_TO_BEAN_CENTER,
  ROBOT_ROUTE_FINISH_IN_NUMBER_ZONE,
  ROBOT_ROUTE_COUNT
} RobotRouteId;

typedef enum
{
  ROBOT_ROUTE_POLICY_DIRECT_SURVEY = 0,
  ROBOT_ROUTE_POLICY_REQUIRE_CENTER_PASS,
  ROBOT_ROUTE_POLICY_FINISH
} RobotRoutePolicy;

typedef struct
{
  WorldStationId expected_station;
  int16_t distance_mm;
  int16_t turn_deg;
  uint16_t speed_rpm;
  uint16_t timeout_ms;
  uint8_t calibrated;
} RobotRouteSegment;

typedef struct
{
  RobotRouteId id;
  RobotRoutePolicy policy;
  RobotRouteSegment *segments;
  uint8_t count;
} RobotRoute;

void RobotRoutes_Init(void);
const RobotRoute *RobotRoutes_Get(RobotRouteId id);
uint8_t RobotRoutes_SetSegmentCalibration(RobotRouteId id, uint8_t index,
                                          int16_t distance_mm, int16_t turn_deg,
                                          uint16_t speed_rpm, uint16_t timeout_ms);
uint8_t RobotRoutes_IsCalibrated(RobotRouteId id);
uint8_t RobotRoutes_HasRequiredCenterPass(RobotRouteId id);

#endif
