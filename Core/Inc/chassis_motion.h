#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>

#define CHASSIS_SIDE_ORIGIN 0x01U /* PD8控制M1/M2。 */
#define CHASSIS_SIDE_FAR    0x02U /* PB12控制M3/M4。 */
#define CHASSIS_SIDE_ALL    (CHASSIS_SIDE_ORIGIN | CHASSIS_SIDE_FAR)

typedef enum
{
  CHASSIS_ALIGNMENT_NONE = 0,
  CHASSIS_ALIGNMENT_SETTLING,
  CHASSIS_ALIGNMENT_RETURNING
} ChassisAlignmentState;

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
void ChassisMotion_AdjustAlignTimeout(int16_t delta_ms);
uint16_t ChassisMotion_GetAlignTimeoutMs(void);

void ChassisMotion_Stop(void);
/* 启动到“下一组挡板”的路线段。distance_mm只用于方向、超时和宽松
 * 里程合理性校验；实际停车及到站判定始终由两侧光电门完成。 */
uint8_t ChassisMotion_StartRouteSegment(int16_t distance_mm, int16_t turn_deg,
                                        uint16_t speed_rpm, uint16_t timeout_ms);
uint8_t ChassisMotion_StartAlignedRouteSegment(int16_t distance_mm,
                                               int16_t turn_deg,
                                               uint16_t speed_rpm,
                                               uint16_t timeout_ms);
/* 连续通过若干组Y挡板。单侧触发只锁存事件、不单边停车；两侧同组确认后：
 * 中途地标保持速度连续通过，目标地标四轮同时停车。 */
uint8_t ChassisMotion_StartRouteThroughLandmarks(int16_t distance_mm,
                                                 uint8_t landmark_count,
                                                 uint16_t speed_rpm,
                                                 uint16_t timeout_ms);
uint8_t ChassisMotion_StartAlignedRouteThroughLandmarks(int16_t distance_mm,
                                                        uint8_t landmark_count,
                                                        uint16_t speed_rpm,
                                                        uint16_t timeout_ms);
/* 已越过目标挡板后的低速回退对齐：启动时已遮挡的一侧直接计为已对齐，
 * 其余侧重新进入遮挡后，双侧同时停车。 */
uint8_t ChassisMotion_StartPhotoBlockedAlignment(uint8_t reverse,
                                                 uint16_t speed_rpm,
                                                 uint16_t timeout_ms);
/* 只按光电地标计数导航；reverse决定Y方向，编码器不参与距离窗口判断。 */
uint8_t ChassisMotion_StartPhotoLandmarkRoute(uint8_t reverse,
                                              uint8_t landmark_count,
                                              uint16_t speed_rpm,
                                              uint16_t timeout_ms);
uint8_t ChassisMotion_StartAlignedPhotoLandmarkRoute(uint8_t reverse,
                                                     uint8_t landmark_count,
                                                     uint16_t speed_rpm,
                                                     uint16_t timeout_ms);
uint8_t ChassisMotion_IsRouteSegmentDone(void);
uint8_t ChassisMotion_DidRouteSegmentFail(void);
uint8_t ChassisMotion_GetPassedLandmarkCount(uint8_t side);
ChassisAlignmentState ChassisMotion_GetAlignmentState(void);

#endif
