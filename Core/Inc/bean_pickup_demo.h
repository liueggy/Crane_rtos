#ifndef BEAN_PICKUP_DEMO_H
#define BEAN_PICKUP_DEMO_H

#include <stdint.h>

typedef enum
{
  BEAN_PICKUP_DEMO_IDLE = 0,
  BEAN_PICKUP_DEMO_HOME_Z_SETTLE,
  BEAN_PICKUP_DEMO_HOME_Z_TOP,
  BEAN_PICKUP_DEMO_HOME_X_SETTLE,
  BEAN_PICKUP_DEMO_HOME_X_RIGHT,
  BEAN_PICKUP_DEMO_CLEAR_X_SETTLE,
  BEAN_PICKUP_DEMO_CLEAR_X_LIMIT,
  BEAN_PICKUP_DEMO_READY,
  BEAN_PICKUP_DEMO_PREPARE_Z_SETTLE,
  BEAN_PICKUP_DEMO_PREPARE_Z,
  BEAN_PICKUP_DEMO_OPEN_GRIPPER,
  BEAN_PICKUP_DEMO_MOVE_X_SETTLE,
  BEAN_PICKUP_DEMO_MOVE_X,
  BEAN_PICKUP_DEMO_DESCEND,
  BEAN_PICKUP_DEMO_CLOSE_GRIPPER,
  BEAN_PICKUP_DEMO_RAISE_SETTLE,
  BEAN_PICKUP_DEMO_RAISE,
  BEAN_PICKUP_DEMO_COMPLETE,
  BEAN_PICKUP_DEMO_FAULT,
} BeanPickupDemoState;

void BeanPickupDemo_Init(void);
void BeanPickupDemo_Process(void);
void BeanPickupDemo_SelectPosition(uint8_t position);
void BeanPickupDemo_HandlePower(void);
/* 坐标已由本次上电回零建立时，直接启动指定物理位置抓取。 */
uint8_t BeanPickupDemo_StartReferencedPickup(uint8_t position);
void BeanPickupDemo_Abort(void);

BeanPickupDemoState BeanPickupDemo_GetState(void);
uint8_t BeanPickupDemo_GetSelectedPosition(void);
uint8_t BeanPickupDemo_IsRunning(void);
uint8_t BeanPickupDemo_IsStartReady(void);

#endif
