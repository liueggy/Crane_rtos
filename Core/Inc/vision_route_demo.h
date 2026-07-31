#ifndef VISION_ROUTE_DEMO_H
#define VISION_ROUTE_DEMO_H

#include "k230_link.h"
#include "vision_survey.h"

#include <stdint.h>

typedef enum
{
  VISION_ROUTE_DEMO_IDLE = 0,
  VISION_ROUTE_DEMO_LIFT_Z,
  VISION_ROUTE_DEMO_HOME_X,
  VISION_ROUTE_DEMO_ALIGN_X,
  VISION_ROUTE_DEMO_MOVE_NUMBER,
  VISION_ROUTE_DEMO_SCAN_NUMBER_START,
  VISION_ROUTE_DEMO_SCAN_NUMBER_ROW,
  VISION_ROUTE_DEMO_SCAN_NUMBER_SIDE,
  VISION_ROUTE_DEMO_RESCAN_NUMBER_MOVE,
  VISION_ROUTE_DEMO_RESCAN_NUMBER_DWELL,
  VISION_ROUTE_DEMO_RESCAN_NUMBER_SIDE,
  VISION_ROUTE_DEMO_MOVE_BEAN,
  VISION_ROUTE_DEMO_SCAN_BEAN_A,
  VISION_ROUTE_DEMO_SCAN_BEAN_B,
  VISION_ROUTE_DEMO_POST_PREPARE,
  VISION_ROUTE_DEMO_POST_MOVE_PICK,
  VISION_ROUTE_DEMO_POST_PICK,
  VISION_ROUTE_DEMO_POST_MOVE_DROP,
  VISION_ROUTE_DEMO_POST_DROP,
  VISION_ROUTE_DEMO_POST_RETURN_START,
  VISION_ROUTE_DEMO_POST_CENTER_X,
  VISION_ROUTE_DEMO_POST_HOME_Z,
  VISION_ROUTE_DEMO_COMPLETE,
  VISION_ROUTE_DEMO_FAULT
} VisionRouteDemoState;

void VisionRouteDemo_Init(void);
void VisionRouteDemo_Process(void);
void VisionRouteDemo_ToggleRunning(void);
/* 视觉页调试入口：横排数字采用到点停车、稳定后采集。 */
uint8_t VisionRouteDemo_StartDebug(void);
/* 正式比赛入口：复用已完成实机调试的到点停车、多数字符锁存逻辑。 */
uint8_t VisionRouteDemo_StartCompetition(void);
/* 已位于豆子识别地标时，对A/B姿态再采集一轮并保留原可信结果。 */
uint8_t VisionRouteDemo_StartBeanRescan(void);
void VisionRouteDemo_Abort(void);

VisionRouteDemoState VisionRouteDemo_GetState(void);
uint8_t VisionRouteDemo_IsRunning(void);
/* result->task指定读取数字或豆子锁存结果；两组结果互不覆盖。 */
uint8_t VisionRouteDemo_GetDisplayResult(K230VisionResult *result);
uint8_t VisionRouteDemo_GetResultWarning(void);
uint8_t VisionRouteDemo_IsComplete(void);
uint8_t VisionRouteDemo_IsRetrying(void);
const VisionSurveyMap *VisionRouteDemo_GetSurveyMap(void);
uint8_t VisionRouteDemo_GetNumberTrustedMask(void);
uint8_t VisionRouteDemo_GetBeanTrustedMask(void);

#endif
