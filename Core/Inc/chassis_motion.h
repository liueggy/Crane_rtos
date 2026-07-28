#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>

#define CHASSIS_SIDE_ORIGIN 0x01U /* PD8控制M1/M2。 */
#define CHASSIS_SIDE_FAR    0x02U /* PB12控制M3/M4。 */
#define CHASSIS_SIDE_ALL    (CHASSIS_SIDE_ORIGIN | CHASSIS_SIDE_FAR)

void ChassisMotion_Init(void);
void ChassisMotion_TaskStep(void);

void ChassisMotion_ToggleRunning(void);
void ChassisMotion_SetRunning(uint8_t running);
void ChassisMotion_StopManual(void);
uint8_t ChassisMotion_IsRunning(void);
uint8_t ChassisMotion_IsPhotoStopArmed(void);
uint8_t ChassisMotion_GetPhotoTriggerMask(void);
uint8_t ChassisMotion_GetRunningSideMask(void);
/* 仅供受控标定流程临时旁路底盘光电停车；正常运行必须保持开启。 */
void ChassisMotion_SetPhotoStopEnabled(uint8_t enabled);

uint8_t ChassisMotion_SetDirection(uint8_t reverse);
uint8_t ChassisMotion_IsDirectionReverse(void);
uint8_t ChassisMotion_ToggleClosedLoop(void);
uint8_t ChassisMotion_SetClosedLoop(uint8_t enabled);
uint8_t ChassisMotion_IsClosedLoop(void);
void ChassisMotion_AdjustTargetRpm(int16_t delta_rpm);
void ChassisMotion_SetTargetRpm(int16_t target_rpm);
int16_t ChassisMotion_GetTargetRpm(void);

void ChassisMotion_Stop(void);
/* 启动到“下一组挡板”的路线段。distance_mm只用于方向、超时和宽松
 * 里程合理性校验；实际停车及到站判定始终由两侧光电门完成。 */
uint8_t ChassisMotion_StartRouteSegment(int16_t distance_mm, int16_t turn_deg,
                                        uint16_t speed_rpm, uint16_t timeout_ms);
uint8_t ChassisMotion_IsRouteSegmentDone(void);
uint8_t ChassisMotion_DidRouteSegmentFail(void);

#endif
