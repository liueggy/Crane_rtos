#include "robot_routes.h"

/* 2026-07-28实测的相邻Y地标间距，+Y朝数字区。
 * 数值只用于方向、超时和宽松里程校验；各段边界由光电门触发确定。 */
#define DIST_BEAN_AC_TO_B_MM                 257
#define DIST_BEAN_B_TO_UPPER_SCAN_MM         327
#define DIST_UPPER_SCAN_TO_LOWER_MM          611
#define DIST_UPPER_LOWER_TO_START_MM         858
#define DIST_START_TO_LOWER_UPPER_MM         407
#define DIST_LOWER_UPPER_TO_NUMBER_SCAN_MM   590
#define DIST_NUMBER_SCAN_TO_SIDE_15_MM       435

/* 理论运行时间外增加3秒，覆盖加减速、两侧分别触发及机械差异。 */
#define ROUTE_TIMEOUT_MS(distance_mm, speed_rpm) \
  ((uint16_t)((((uint32_t)(distance_mm) * 60000U) / \
                ((uint32_t)(speed_rpm) * 264U)) + 3000U))

/* 识别路线允许直行；搬运路线必须经过起点并完成X向交叉换边。 */
static RobotRouteSegment g_survey_to_number[] = {
  {WORLD_STATION_LOWER_OBSTACLE_UPPER, DIST_START_TO_LOWER_UPPER_MM, 0,
   ROBOT_ROUTE_CRUISE_RPM,
   ROUTE_TIMEOUT_MS(DIST_START_TO_LOWER_UPPER_MM, ROBOT_ROUTE_CRUISE_RPM), 1U},
  {WORLD_STATION_LOWER_OBSTACLE_NUMBER_SCAN, DIST_LOWER_UPPER_TO_NUMBER_SCAN_MM, 0,
   ROBOT_ROUTE_APPROACH_RPM,
   ROUTE_TIMEOUT_MS(DIST_LOWER_UPPER_TO_NUMBER_SCAN_MM, ROBOT_ROUTE_APPROACH_RPM), 1U},
};

static RobotRouteSegment g_survey_number_to_bean[] = {
  {WORLD_STATION_LOWER_OBSTACLE_UPPER, -DIST_LOWER_UPPER_TO_NUMBER_SCAN_MM, 0,
   ROBOT_ROUTE_CRUISE_RPM,
   ROUTE_TIMEOUT_MS(DIST_LOWER_UPPER_TO_NUMBER_SCAN_MM, ROBOT_ROUTE_CRUISE_RPM), 1U},
  {WORLD_STATION_START, -DIST_START_TO_LOWER_UPPER_MM, 0,
   ROBOT_ROUTE_CRUISE_RPM,
   ROUTE_TIMEOUT_MS(DIST_START_TO_LOWER_UPPER_MM, ROBOT_ROUTE_CRUISE_RPM), 1U},
  {WORLD_STATION_UPPER_OBSTACLE_LOWER, -DIST_UPPER_LOWER_TO_START_MM, 0,
   ROBOT_ROUTE_CRUISE_RPM,
   ROUTE_TIMEOUT_MS(DIST_UPPER_LOWER_TO_START_MM, ROBOT_ROUTE_CRUISE_RPM), 1U},
  {WORLD_STATION_UPPER_OBSTACLE_BEAN_SCAN, -DIST_UPPER_SCAN_TO_LOWER_MM, 0,
   ROBOT_ROUTE_APPROACH_RPM,
   ROUTE_TIMEOUT_MS(DIST_UPPER_SCAN_TO_LOWER_MM, ROBOT_ROUTE_APPROACH_RPM), 1U},
};

static RobotRouteSegment g_transport_bean_to_number[] = {
  {WORLD_STATION_UPPER_OBSTACLE_LOWER, DIST_UPPER_SCAN_TO_LOWER_MM, 0,
   ROBOT_ROUTE_APPROACH_RPM,
   ROUTE_TIMEOUT_MS(DIST_UPPER_SCAN_TO_LOWER_MM, ROBOT_ROUTE_APPROACH_RPM), 1U},
  {WORLD_STATION_START, DIST_UPPER_LOWER_TO_START_MM, 0,
   ROBOT_ROUTE_CRUISE_RPM,
   ROUTE_TIMEOUT_MS(DIST_UPPER_LOWER_TO_START_MM, ROBOT_ROUTE_CRUISE_RPM), 1U},
  {WORLD_STATION_LOWER_OBSTACLE_UPPER, DIST_START_TO_LOWER_UPPER_MM, 0,
   ROBOT_ROUTE_CRUISE_RPM,
   ROUTE_TIMEOUT_MS(DIST_START_TO_LOWER_UPPER_MM, ROBOT_ROUTE_CRUISE_RPM), 1U},
  {WORLD_STATION_LOWER_OBSTACLE_NUMBER_SCAN, DIST_LOWER_UPPER_TO_NUMBER_SCAN_MM, 0,
   ROBOT_ROUTE_APPROACH_RPM,
   ROUTE_TIMEOUT_MS(DIST_LOWER_UPPER_TO_NUMBER_SCAN_MM, ROBOT_ROUTE_APPROACH_RPM), 1U},
};

static RobotRouteSegment g_transport_number_to_bean[] = {
  {WORLD_STATION_LOWER_OBSTACLE_UPPER, -DIST_LOWER_UPPER_TO_NUMBER_SCAN_MM, 0,
   ROBOT_ROUTE_APPROACH_RPM,
   ROUTE_TIMEOUT_MS(DIST_LOWER_UPPER_TO_NUMBER_SCAN_MM, ROBOT_ROUTE_APPROACH_RPM), 1U},
  {WORLD_STATION_START, -DIST_START_TO_LOWER_UPPER_MM, 0,
   ROBOT_ROUTE_CRUISE_RPM,
   ROUTE_TIMEOUT_MS(DIST_START_TO_LOWER_UPPER_MM, ROBOT_ROUTE_CRUISE_RPM), 1U},
  {WORLD_STATION_UPPER_OBSTACLE_LOWER, -DIST_UPPER_LOWER_TO_START_MM, 0,
   ROBOT_ROUTE_CRUISE_RPM,
   ROUTE_TIMEOUT_MS(DIST_UPPER_LOWER_TO_START_MM, ROBOT_ROUTE_CRUISE_RPM), 1U},
  {WORLD_STATION_UPPER_OBSTACLE_BEAN_SCAN, -DIST_UPPER_SCAN_TO_LOWER_MM, 0,
   ROBOT_ROUTE_APPROACH_RPM,
   ROUTE_TIMEOUT_MS(DIST_UPPER_SCAN_TO_LOWER_MM, ROBOT_ROUTE_APPROACH_RPM), 1U},
};

static RobotRouteSegment g_finish_number_zone[] = {
  {WORLD_STATION_LOWER_OBSTACLE_NUMBER_SCAN, 0, 0, ROBOT_ROUTE_APPROACH_RPM, 0U, 0U},
};

static RobotRoute g_routes[ROBOT_ROUTE_COUNT] = {
  {ROBOT_ROUTE_SURVEY_START_TO_NUMBER, ROBOT_ROUTE_POLICY_DIRECT_SURVEY,
   g_survey_to_number, 2U},
  {ROBOT_ROUTE_SURVEY_NUMBER_TO_BEAN_DIRECT, ROBOT_ROUTE_POLICY_DIRECT_SURVEY,
   g_survey_number_to_bean, 4U},
  {ROBOT_ROUTE_TRANSPORT_BEAN_TO_NUMBER_CENTER,
   ROBOT_ROUTE_POLICY_REQUIRE_CENTER_PASS, g_transport_bean_to_number, 4U},
  {ROBOT_ROUTE_TRANSPORT_NUMBER_TO_BEAN_CENTER,
   ROBOT_ROUTE_POLICY_REQUIRE_CENTER_PASS, g_transport_number_to_bean, 4U},
  {ROBOT_ROUTE_FINISH_IN_NUMBER_ZONE, ROBOT_ROUTE_POLICY_FINISH,
   g_finish_number_zone, 1U},
};

static uint8_t IsPrecisionStation(WorldStationId station)
{
  return (station == WORLD_STATION_BEAN_AC_PICK) ||
         (station == WORLD_STATION_BEAN_B_PICK) ||
         (station == WORLD_STATION_UPPER_OBSTACLE_BEAN_SCAN) ||
         (station == WORLD_STATION_LOWER_OBSTACLE_NUMBER_SCAN) ||
         (station == WORLD_STATION_NUMBER_SIDE_15) ||
         (station == WORLD_STATION_NUMBER_BOTTOM_234);
}

void RobotRoutes_Init(void)
{
  /* 主识别和搬运通道已装入实测值；箱区端部路线等待剩余距离。 */
}

const RobotRoute *RobotRoutes_Get(RobotRouteId id)
{
  return (id < ROBOT_ROUTE_COUNT) ? &g_routes[id] : 0;
}

uint8_t RobotRoutes_SetSegmentCalibration(RobotRouteId id, uint8_t index,
                                          int16_t distance_mm, int16_t turn_deg,
                                          uint16_t speed_rpm, uint16_t timeout_ms)
{
  if ((id >= ROBOT_ROUTE_COUNT) || (index >= g_routes[id].count) ||
      (distance_mm == 0) || (speed_rpm == 0U) ||
      (speed_rpm > ROBOT_ROUTE_CRUISE_RPM) || (timeout_ms < 100U)) return 0U;
  RobotRouteSegment *segment = &g_routes[id].segments[index];
  if (IsPrecisionStation(segment->expected_station) &&
      (speed_rpm > ROBOT_ROUTE_APPROACH_RPM)) return 0U;
  segment->distance_mm = distance_mm;
  segment->turn_deg = turn_deg;
  segment->speed_rpm = speed_rpm;
  segment->timeout_ms = timeout_ms;
  segment->calibrated = 1U;
  return 1U;
}

uint8_t RobotRoutes_IsCalibrated(RobotRouteId id)
{
  if (id >= ROBOT_ROUTE_COUNT) return 0U;
  for (uint8_t i = 0U; i < g_routes[id].count; ++i)
    if (!g_routes[id].segments[i].calibrated) return 0U;
  return 1U;
}

uint8_t RobotRoutes_HasRequiredCenterPass(RobotRouteId id)
{
  if (id >= ROBOT_ROUTE_COUNT) return 0U;
  if (g_routes[id].policy != ROBOT_ROUTE_POLICY_REQUIRE_CENTER_PASS) return 1U;
  for (uint8_t i = 0U; i < g_routes[id].count; ++i)
    if (g_routes[id].segments[i].expected_station == WORLD_STATION_START) return 1U;
  return 0U;
}
