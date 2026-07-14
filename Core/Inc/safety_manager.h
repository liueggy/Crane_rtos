#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <stdint.h>

typedef enum
{
  SAFETY_LIMIT_X_MIN = (1UL << 0),
  SAFETY_LIMIT_X_MAX = (1UL << 1),
  SAFETY_LIMIT_Z_MAX = (1UL << 2),
  SAFETY_LIMIT_Z_MIN = (1UL << 3),
  SAFETY_FAULT_ESTOP = (1UL << 8),
  SAFETY_FAULT_LIMIT = (1UL << 9)
} SafetyFlag;

void SafetyManager_Init(void);
void SafetyManager_HandleExti(uint16_t gpio_pin);
void SafetyManager_TriggerEstop(void);
uint32_t SafetyManager_GetFlags(void);
uint8_t SafetyManager_IsEstopActive(void);

#endif
