#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum
{
  UI_PAGE_OVERVIEW = 0,
  UI_PAGE_STEPPER,
  UI_PAGE_MOTOR_SPEED,
  UI_PAGE_MOTOR_TUNING,
  UI_PAGE_ENCODER,
  UI_PAGE_SYSTEM,
  UI_PAGE_COUNT
} UiPage;

/* 初始化 OLED 和页面状态。 */
void UiManager_Init(I2C_HandleTypeDef *i2c);
/* 页面循环和指定页面接口，供按键或外接面板调用。 */
void UiManager_NextPage(void);
void UiManager_SetPage(UiPage page);
UiPage UiManager_GetPage(void);
void UiManager_Render(void);

#endif
