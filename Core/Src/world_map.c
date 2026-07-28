#include "world_map.h"

#include "app_config.h"

#include <string.h>

#define CAL_MISSING_STATIONS (1UL << 0)
#define CAL_MISSING_SLOTS    (1UL << 1)
#define CAL_MISSING_SCANS    (1UL << 2)

static WorldStation g_stations[WORLD_STATION_COUNT];
static WorldSlotPose g_slots[WORLD_SLOT_COUNT];
static WorldScanPose g_scans[WORLD_SCAN_COUNT];
static WorldPose g_pose;

static const WorldStationId k_slot_stations[WORLD_SLOT_COUNT] = {
  WORLD_STATION_BEAN_AC_PICK,
  WORLD_STATION_BEAN_AC_PICK,
  WORLD_STATION_BEAN_B_PICK,
  WORLD_STATION_NUMBER_BOTTOM_234,
  WORLD_STATION_NUMBER_BOTTOM_234,
  WORLD_STATION_NUMBER_BOTTOM_234,
  WORLD_STATION_NUMBER_SIDE_15,
  WORLD_STATION_NUMBER_SIDE_15,
};

static const WorldStationId k_scan_stations[WORLD_SCAN_COUNT] = {
  WORLD_STATION_LOWER_OBSTACLE_NUMBER_SCAN,
  WORLD_STATION_LOWER_OBSTACLE_NUMBER_SCAN,
  WORLD_STATION_UPPER_OBSTACLE_BEAN_SCAN,
};

void WorldMap_Init(void)
{
  memset(g_stations, 0, sizeof(g_stations));
  memset(g_slots, 0, sizeof(g_slots));
  memset(g_scans, 0, sizeof(g_scans));
  memset(&g_pose, 0, sizeof(g_pose));

  for (uint8_t i = 0U; i < WORLD_STATION_COUNT; ++i)
  {
    g_stations[i].id = (WorldStationId)i;
    g_stations[i].world_y_mm = WORLD_MAP_UNCALIBRATED;
  }
  /* 场地中心起点的名义坐标已知，挡板对应Y坐标仍需实机测量。 */
  g_stations[WORLD_STATION_START].world_y_mm = 2000;
  g_stations[WORLD_STATION_START].calibrated = 1U;

  for (uint8_t i = 0U; i < WORLD_SLOT_COUNT; ++i)
  {
    g_slots[i].id = (WorldSlotId)i;
    g_slots[i].station = k_slot_stations[i];
    g_slots[i].gantry_x_pulses = WORLD_MAP_UNCALIBRATED;
    g_slots[i].action_z_pulses = WORLD_MAP_UNCALIBRATED;
    g_slots[i].tool_yaw_degrees = SLOT_YAW_INITIAL_DEGREES;
  }
  /* 已确认的数字箱X坐标直接装入地图；抓放Z高度仍保持未标定。 */
  g_slots[WORLD_SLOT_NUMBER_BOTTOM_LEFT].gantry_x_pulses =
      NUMBER_SLOT_BOTTOM_LEFT_X_PULSES;
  g_slots[WORLD_SLOT_NUMBER_BOTTOM_CENTER].gantry_x_pulses =
      NUMBER_SLOT_BOTTOM_CENTER_X_PULSES;
  g_slots[WORLD_SLOT_NUMBER_BOTTOM_RIGHT].gantry_x_pulses =
      NUMBER_SLOT_BOTTOM_RIGHT_X_PULSES;
  g_slots[WORLD_SLOT_NUMBER_OFFSET_LEFT].gantry_x_pulses =
      NUMBER_SLOT_OFFSET_LEFT_X_PULSES;
  g_slots[WORLD_SLOT_NUMBER_OFFSET_RIGHT].gantry_x_pulses =
      NUMBER_SLOT_OFFSET_RIGHT_X_PULSES;
  g_slots[WORLD_SLOT_BEAN_TOP_LEFT].gantry_x_pulses =
      BEAN_SLOT_TOP_LEFT_X_PULSES;
  g_slots[WORLD_SLOT_BEAN_TOP_RIGHT].gantry_x_pulses =
      BEAN_SLOT_TOP_RIGHT_X_PULSES;
  g_slots[WORLD_SLOT_BEAN_OFFSET].gantry_x_pulses =
      BEAN_SLOT_OFFSET_X_PULSES;
  /* 三档抓取高度固定对应场地上的三个物理豆子箱位。 */
  g_slots[WORLD_SLOT_BEAN_OFFSET].action_z_pulses =
      BEAN_PICKUP_Z_LEVEL_1_PULSES;
  g_slots[WORLD_SLOT_BEAN_TOP_RIGHT].action_z_pulses =
      BEAN_PICKUP_Z_LEVEL_2_PULSES;
  g_slots[WORLD_SLOT_BEAN_TOP_LEFT].action_z_pulses =
      BEAN_PICKUP_Z_LEVEL_3_PULSES;
  g_slots[WORLD_SLOT_BEAN_OFFSET].calibrated = 1U;
  g_slots[WORLD_SLOT_BEAN_TOP_RIGHT].calibrated = 1U;
  g_slots[WORLD_SLOT_BEAN_TOP_LEFT].calibrated = 1U;
  /* 五个数字箱使用同一个已确认放豆高度。 */
  for (uint8_t i = WORLD_SLOT_NUMBER_BOTTOM_LEFT; i < WORLD_SLOT_COUNT; ++i)
  {
    g_slots[i].action_z_pulses = NUMBER_DROP_Z_PULSES;
    g_slots[i].calibrated = 1U;
  }
  /* 数字区底部横排三箱需横向放豆，两侧凸出箱及豆子箱保持初始姿态。 */
  g_slots[WORLD_SLOT_NUMBER_BOTTOM_LEFT].tool_yaw_degrees =
      NUMBER_BOTTOM_ROW_YAW_DEGREES;
  g_slots[WORLD_SLOT_NUMBER_BOTTOM_CENTER].tool_yaw_degrees =
      NUMBER_BOTTOM_ROW_YAW_DEGREES;
  g_slots[WORLD_SLOT_NUMBER_BOTTOM_RIGHT].tool_yaw_degrees =
      NUMBER_BOTTOM_ROW_YAW_DEGREES;
  for (uint8_t i = 0U; i < WORLD_SCAN_COUNT; ++i)
  {
    g_scans[i].id = (WorldScanPoseId)i;
    g_scans[i].station = k_scan_stations[i];
    g_scans[i].gantry_x_pulses = WORLD_MAP_UNCALIBRATED;
    g_scans[i].gantry_z_pulses = WORLD_MAP_UNCALIBRATED;
  }
  WorldMap_SetPoseAtStart();
}

const WorldStation *WorldMap_GetStation(WorldStationId id)
{
  return (id < WORLD_STATION_COUNT) ? &g_stations[id] : 0;
}

const WorldSlotPose *WorldMap_GetSlot(WorldSlotId id)
{
  return (id < WORLD_SLOT_COUNT) ? &g_slots[id] : 0;
}

const WorldScanPose *WorldMap_GetScanPose(WorldScanPoseId id)
{
  return (id < WORLD_SCAN_COUNT) ? &g_scans[id] : 0;
}

const WorldPose *WorldMap_GetPose(void)
{
  return &g_pose;
}

uint8_t WorldMap_SetStationY(WorldStationId id, int32_t world_y_mm)
{
  if ((id >= WORLD_STATION_COUNT) || (world_y_mm < 0) || (world_y_mm > 4000)) return 0U;
  g_stations[id].world_y_mm = world_y_mm;
  g_stations[id].calibrated = 1U;
  return 1U;
}

uint8_t WorldMap_SetSlotPose(WorldSlotId id, int32_t x_pulses,
                             int32_t action_z_pulses)
{
  if ((id >= WORLD_SLOT_COUNT) || (x_pulses < 0) ||
      (x_pulses > (int32_t)STEPPER_X_TRAVEL_PULSES) ||
      (action_z_pulses < 0) ||
      (action_z_pulses > (int32_t)STEPPER_Z_TRAVEL_PULSES)) return 0U;
  g_slots[id].gantry_x_pulses = x_pulses;
  g_slots[id].action_z_pulses = action_z_pulses;
  g_slots[id].calibrated = 1U;
  return 1U;
}

uint8_t WorldMap_SetSlotX(WorldSlotId id, int32_t x_pulses)
{
  if ((id >= WORLD_SLOT_COUNT) || (x_pulses < 0) ||
      (x_pulses > (int32_t)STEPPER_X_TRAVEL_PULSES)) return 0U;
  g_slots[id].gantry_x_pulses = x_pulses;
  g_slots[id].calibrated =
      (g_slots[id].action_z_pulses != WORLD_MAP_UNCALIBRATED) ? 1U : 0U;
  return 1U;
}

uint8_t WorldMap_SetSlotZ(WorldSlotId id, int32_t action_z_pulses)
{
  if ((id >= WORLD_SLOT_COUNT) || (action_z_pulses < 0) ||
      (action_z_pulses > (int32_t)STEPPER_Z_TRAVEL_PULSES)) return 0U;
  g_slots[id].action_z_pulses = action_z_pulses;
  g_slots[id].calibrated =
      (g_slots[id].gantry_x_pulses != WORLD_MAP_UNCALIBRATED) ? 1U : 0U;
  return 1U;
}

uint8_t WorldMap_SetScanPose(WorldScanPoseId id, int32_t x_pulses,
                             int32_t z_pulses, uint16_t yaw_degrees,
                             uint16_t tilt_pulse_us, uint16_t settle_ms)
{
  if ((id >= WORLD_SCAN_COUNT) || (x_pulses < 0) ||
      (x_pulses > (int32_t)STEPPER_X_TRAVEL_PULSES) || (z_pulses < 0) ||
      (z_pulses > (int32_t)STEPPER_Z_TRAVEL_PULSES) ||
      (yaw_degrees > 270U) || (tilt_pulse_us < 500U) ||
      (tilt_pulse_us > 2500U) || (settle_ms < 100U)) return 0U;
  g_scans[id].gantry_x_pulses = x_pulses;
  g_scans[id].gantry_z_pulses = z_pulses;
  g_scans[id].yaw_degrees = yaw_degrees;
  g_scans[id].tilt_pulse_us = tilt_pulse_us;
  g_scans[id].settle_ms = settle_ms;
  g_scans[id].calibrated = 1U;
  return 1U;
}

void WorldMap_SetPoseAtStart(void)
{
  g_pose.x_pulses = (int32_t)(STEPPER_X_TRAVEL_PULSES / 2U);
  g_pose.y_mm = 2000;
  g_pose.z_pulses = (int32_t)STEPPER_Z_TRAVEL_PULSES;
  g_pose.station = WORLD_STATION_START;
  /* 比赛由人工按规定姿态放置；该坐标仍应在首次挡板处复核。 */
  g_pose.x_valid = 1U;
  g_pose.y_valid = 1U;
  g_pose.z_valid = 1U;
}

void WorldMap_SetKnownStation(WorldStationId station)
{
  if ((station >= WORLD_STATION_COUNT) || !g_stations[station].calibrated) return;
  g_pose.station = station;
  g_pose.y_mm = g_stations[station].world_y_mm;
  g_pose.y_valid = 1U;
}

void WorldMap_SetAxisPosition(int32_t x_pulses, uint8_t x_valid,
                              int32_t z_pulses, uint8_t z_valid)
{
  g_pose.x_pulses = x_pulses;
  g_pose.z_pulses = z_pulses;
  g_pose.x_valid = x_valid ? 1U : 0U;
  g_pose.z_valid = z_valid ? 1U : 0U;
}

uint8_t WorldMap_IsStartupMotionCalibrated(void)
{
  /* Z先抬到离地75mm，X光电回零后使用已标定的2号箱X坐标。 */
  return 1U;
}

uint8_t WorldMap_IsSurveyCalibrated(void)
{
  return g_stations[WORLD_STATION_LOWER_OBSTACLE_NUMBER_SCAN].calibrated &&
         g_stations[WORLD_STATION_UPPER_OBSTACLE_BEAN_SCAN].calibrated &&
         g_scans[WORLD_SCAN_NUMBER_A].calibrated &&
         g_scans[WORLD_SCAN_NUMBER_B].calibrated &&
         g_scans[WORLD_SCAN_BEAN_C].calibrated;
}

uint8_t WorldMap_IsTaskMotionCalibrated(void)
{
  for (uint8_t i = 0U; i < WORLD_SLOT_COUNT; ++i)
    if (!g_slots[i].calibrated) return 0U;
  return 1U;
}

uint32_t WorldMap_GetMissingCalibrationMask(void)
{
  uint32_t mask = 0U;
  for (uint8_t i = 0U; i < WORLD_STATION_COUNT; ++i)
    if (!g_stations[i].calibrated) mask |= CAL_MISSING_STATIONS;
  for (uint8_t i = 0U; i < WORLD_SLOT_COUNT; ++i)
    if (!g_slots[i].calibrated) mask |= CAL_MISSING_SLOTS;
  for (uint8_t i = 0U; i < WORLD_SCAN_COUNT; ++i)
    if (!g_scans[i].calibrated) mask |= CAL_MISSING_SCANS;
  return mask;
}

uint32_t WorldMap_GetStartupLiftPulses(void)
{
  return RECOGNITION_LIFT_PULSES;
}

uint32_t WorldMap_GetNumber2AlignmentPulses(void)
{
  return NUMBER_SLOT_BOTTOM_RIGHT_X_PULSES;
}

int32_t WorldMap_GetTransportSafeZPulses(void)
{
  return (int32_t)RECOGNITION_TRAVEL_Z_PULSES;
}

int32_t WorldMap_GetPickupPrepZPulses(void)
{
  return (int32_t)BEAN_PICKUP_PREP_Z_PULSES;
}
