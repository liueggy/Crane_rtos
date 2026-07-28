#ifndef BOX_CALIBRATION_H
#define BOX_CALIBRATION_H

#include <stdint.h>

void BoxCalibration_Init(void);
void BoxCalibration_SelectSlot(uint8_t slot);
void BoxCalibration_AdjustStep(int8_t direction);
uint8_t BoxCalibration_Jog(uint8_t reverse);
uint8_t BoxCalibration_SetXZero(void);
uint8_t BoxCalibration_SaveCurrent(void);

uint8_t BoxCalibration_GetSelectedSlot(void);
uint32_t BoxCalibration_GetJogStep(void);
int32_t BoxCalibration_GetCurrentX(void);
uint8_t BoxCalibration_IsReferenceValid(void);
uint8_t BoxCalibration_IsSelectedSaved(void);

#endif
