#ifndef INITIALIZATION_DEBUG_H
#define INITIALIZATION_DEBUG_H

#include <stdint.h>

typedef enum
{
  INITIALIZATION_DEBUG_IDLE = 0,
  INITIALIZATION_DEBUG_SEEK_Z_BOTTOM,
  INITIALIZATION_DEBUG_DIRECTION_SETTLE,
  INITIALIZATION_DEBUG_RAISE_Z_TOP,
  INITIALIZATION_DEBUG_HOME_X_RIGHT,
  INITIALIZATION_DEBUG_MOVE_X_CENTER,
  INITIALIZATION_DEBUG_LOWER_Z_BOTTOM,
  INITIALIZATION_DEBUG_COMPLETE,
  INITIALIZATION_DEBUG_FAULT,
} InitializationDebugState;

void InitializationDebug_Init(void);
void InitializationDebug_Process(void);
void InitializationDebug_ToggleRunning(void);
void InitializationDebug_Abort(void);

InitializationDebugState InitializationDebug_GetState(void);
uint8_t InitializationDebug_IsRunning(void);

#endif
