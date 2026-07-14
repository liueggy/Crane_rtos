#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <stdint.h>

typedef enum
{
  ROBOT_STATE_IDLE = 0,
  ROBOT_STATE_SELF_CHECK,
  ROBOT_STATE_VISION_SCAN,
  ROBOT_STATE_TASK_BUILD,
  ROBOT_STATE_MOVE_TO_PICK,
  ROBOT_STATE_PICK_ACTION,
  ROBOT_STATE_LIFT_SAFE,
  ROBOT_STATE_FOLLOW_ROUTE,
  ROBOT_STATE_DROP_ACTION,
  ROBOT_STATE_RETURN_FINISH,
  ROBOT_STATE_FINISHED,
  ROBOT_STATE_FAULT
} RobotState;

typedef struct
{
  uint8_t bean_type;
  uint8_t pick_position;
  uint8_t target_box;
} RobotTransportTask;

void RobotController_Init(void);
void RobotController_RequestStart(void);
void RobotController_RequestAbort(void);
void RobotController_Update(void);
RobotState RobotController_GetState(void);
const RobotTransportTask *RobotController_GetActiveTask(void);

#endif
