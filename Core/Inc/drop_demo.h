#ifndef DROP_DEMO_H
#define DROP_DEMO_H

#include <stdint.h>

typedef enum
{
  DROP_DEMO_IDLE = 0,
  DROP_DEMO_NEED_REFERENCE,
  DROP_DEMO_LIFT_SAFE,
  DROP_DEMO_MOVE_X,
  DROP_DEMO_ROTATE,
  DROP_DEMO_DESCEND,
  DROP_DEMO_RELEASE,
  DROP_DEMO_RAISE,
  DROP_DEMO_RESET_TOOL,
  DROP_DEMO_PAUSED,
  DROP_DEMO_COMPLETE,
  DROP_DEMO_FAULT,
} DropDemoState;

void DropDemo_Init(void);
void DropDemo_Process(void);
void DropDemo_ToggleRunning(void);
void DropDemo_Abort(void);

DropDemoState DropDemo_GetState(void);
uint8_t DropDemo_GetSlotNumber(void);
uint8_t DropDemo_IsRunning(void);

#endif
