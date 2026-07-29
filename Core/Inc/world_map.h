#ifndef WORLD_MAP_H
#define WORLD_MAP_H

#include <stdint.h>

#define WORLD_MAP_UNCALIBRATED INT32_MIN
#define WORLD_BEAN_SLOT_COUNT  3U
#define WORLD_NUMBER_SLOT_COUNT 5U
#define WORLD_PHOTO_LANDMARK_COUNT 9U

typedef enum
{
  /* 实物从豆子端到数字端共9组挡板；A/C和2/3/4分别兼作两端极限。 */
  WORLD_STATION_BEAN_AC_PICK = 0,
  WORLD_STATION_BEAN_B_PICK,
  WORLD_STATION_UPPER_OBSTACLE_BEAN_SCAN,
  WORLD_STATION_UPPER_OBSTACLE_LOWER,
  WORLD_STATION_START,
  WORLD_STATION_LOWER_OBSTACLE_UPPER,
  WORLD_STATION_LOWER_OBSTACLE_NUMBER_SCAN,
  WORLD_STATION_NUMBER_SIDE_15,
  WORLD_STATION_NUMBER_BOTTOM_234,
  WORLD_STATION_COUNT
} WorldStationId;

typedef enum
{
  WORLD_SCAN_NUMBER_A = 0,
  WORLD_SCAN_NUMBER_B,
  WORLD_SCAN_BEAN_C,
  WORLD_SCAN_COUNT
} WorldScanPoseId;

typedef enum
{
  WORLD_SLOT_BEAN_TOP_LEFT = 0,
  WORLD_SLOT_BEAN_TOP_RIGHT,
  WORLD_SLOT_BEAN_OFFSET,
  WORLD_SLOT_NUMBER_BOTTOM_LEFT,
  WORLD_SLOT_NUMBER_BOTTOM_CENTER,
  WORLD_SLOT_NUMBER_BOTTOM_RIGHT,
  WORLD_SLOT_NUMBER_OFFSET_LEFT,
  WORLD_SLOT_NUMBER_OFFSET_RIGHT,
  WORLD_SLOT_COUNT
} WorldSlotId;

typedef struct
{
  WorldStationId id;
  int32_t world_y_mm;
  uint8_t calibrated;
} WorldStation;

typedef struct
{
  WorldSlotId id;
  WorldStationId station;
  int32_t gantry_x_pulses;
  int32_t action_z_pulses;
  uint16_t tool_yaw_degrees;
  uint8_t calibrated;
} WorldSlotPose;

typedef struct
{
  WorldScanPoseId id;
  WorldStationId station;
  int32_t gantry_x_pulses;
  int32_t gantry_z_pulses;
  uint16_t yaw_degrees;
  uint16_t tilt_pulse_us;
  uint16_t settle_ms;
  uint8_t calibrated;
} WorldScanPose;

typedef struct
{
  int32_t x_pulses;
  int32_t y_mm;
  int32_t z_pulses;
  WorldStationId station;
  uint8_t x_valid;
  uint8_t y_valid;
  uint8_t z_valid;
} WorldPose;

void WorldMap_Init(void);
const WorldStation *WorldMap_GetStation(WorldStationId id);
const WorldSlotPose *WorldMap_GetSlot(WorldSlotId id);
const WorldScanPose *WorldMap_GetScanPose(WorldScanPoseId id);
const WorldPose *WorldMap_GetPose(void);

uint8_t WorldMap_SetStationY(WorldStationId id, int32_t world_y_mm);
uint8_t WorldMap_SetSlotPose(WorldSlotId id, int32_t x_pulses,
                             int32_t action_z_pulses);
uint8_t WorldMap_SetSlotX(WorldSlotId id, int32_t x_pulses);
uint8_t WorldMap_SetSlotZ(WorldSlotId id, int32_t action_z_pulses);
uint8_t WorldMap_SetScanPose(WorldScanPoseId id, int32_t x_pulses,
                             int32_t z_pulses, uint16_t yaw_degrees,
                             uint16_t tilt_pulse_us, uint16_t settle_ms);
void WorldMap_SetPoseAtStart(void);
void WorldMap_SetKnownStation(WorldStationId station);
void WorldMap_SetAxisPosition(int32_t x_pulses, uint8_t x_valid,
                              int32_t z_pulses, uint8_t z_valid);

uint8_t WorldMap_IsStartupMotionCalibrated(void);
uint8_t WorldMap_IsSurveyCalibrated(void);
uint8_t WorldMap_IsTaskMotionCalibrated(void);
uint32_t WorldMap_GetMissingCalibrationMask(void);

uint32_t WorldMap_GetStartupLiftPulses(void);
uint32_t WorldMap_GetNumber2AlignmentPulses(void);
int32_t WorldMap_GetTransportSafeZPulses(void);
int32_t WorldMap_GetPickupPrepZPulses(void);

#endif
