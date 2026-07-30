#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum
{
  UI_PAGE_OVERVIEW = 0,
  UI_PAGE_MOTOR_SPEED,
  UI_PAGE_ODOMETRY_CALIBRATION,
  UI_PAGE_STEPPER,
  UI_PAGE_STEPPER_PULSE,
  UI_PAGE_BOX_CALIBRATION,
  UI_PAGE_Z_CALIBRATION,
  UI_PAGE_INITIALIZATION_DEBUG,
  UI_PAGE_BEAN_PICKUP_DEMO,
  UI_PAGE_BEAN_SEQUENCE_DEMO,
  UI_PAGE_XY_WAYPOINT_DEMO,
  UI_PAGE_DROP_DEMO,
  UI_PAGE_SERVO,
  UI_PAGE_VISION,
  UI_PAGE_COMPETITION,
  UI_PAGE_SYSTEM,
  UI_PAGE_COUNT
} UiPage;

/* 初始化 OLED 和页面状态。 */
void UiManager_Init(I2C_HandleTypeDef *i2c);
/* 页面循环和指定页面接口，供按键或外接面板调用。 */
void UiManager_NextPage(void);
void UiManager_PreviousPage(void);
void UiManager_SetPage(UiPage page);
UiPage UiManager_GetPage(void);
void UiManager_Render(void);

#endif
