#ifndef ODOMETRY_CALIBRATION_H
#define ODOMETRY_CALIBRATION_H

#include <stdint.h>

typedef enum
{
  ODOMETRY_FIELD_DISTANCE = 0,
  ODOMETRY_FIELD_SPEED,
} OdometryCalibrationField;

typedef enum
{
  ODOMETRY_CALIBRATION_IDLE = 0,
  ODOMETRY_CALIBRATION_RUNNING,
  ODOMETRY_CALIBRATION_COMPLETE,
  ODOMETRY_CALIBRATION_STOPPED,
  ODOMETRY_CALIBRATION_FAULT,
} OdometryCalibrationState;

void OdometryCalibration_Init(void);
void OdometryCalibration_Process(void);
void OdometryCalibration_ToggleRunning(void);
void OdometryCalibration_Abort(void);
void OdometryCalibration_SelectField(OdometryCalibrationField field);
void OdometryCalibration_AppendDigit(uint8_t digit);
void OdometryCalibration_SetDirection(uint8_t reverse);

OdometryCalibrationState OdometryCalibration_GetState(void);
OdometryCalibrationField OdometryCalibration_GetSelectedField(void);
uint16_t OdometryCalibration_GetTargetDistanceMm(void);
uint16_t OdometryCalibration_GetTargetSpeedRpm(void);
uint8_t OdometryCalibration_GetDirectionReverse(void);
uint32_t OdometryCalibration_GetMeasuredDistanceMmX10(void);
uint32_t OdometryCalibration_GetWheelSpreadMmX10(void);
uint32_t OdometryCalibration_GetWheelDistanceMmX10(uint8_t index);
uint8_t OdometryCalibration_IsRunning(void);

#endif
