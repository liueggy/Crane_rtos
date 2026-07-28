#ifndef Z_CALIBRATION_H
#define Z_CALIBRATION_H

#include <stdint.h>

#define Z_CALIBRATION_TARGET_COUNT 4U

typedef enum
{
  Z_CALIBRATION_PICKUP_1 = 0,
  Z_CALIBRATION_PICKUP_2,
  Z_CALIBRATION_PICKUP_3,
  Z_CALIBRATION_DROP,
} ZCalibrationTarget;

void ZCalibration_Init(void);
void ZCalibration_SelectTarget(uint8_t target);
void ZCalibration_AdjustStep(int8_t direction);
uint8_t ZCalibration_Jog(uint8_t reverse);
uint8_t ZCalibration_SetBottomReference(void);
uint8_t ZCalibration_SaveCurrent(void);

uint8_t ZCalibration_GetSelectedTarget(void);
uint32_t ZCalibration_GetJogStep(void);
int32_t ZCalibration_GetCurrentZ(void);
int32_t ZCalibration_GetSavedZ(uint8_t target);
uint8_t ZCalibration_IsReferenceValid(void);
uint8_t ZCalibration_IsSelectedSaved(void);

#endif
