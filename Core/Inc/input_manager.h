#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include "cmsis_os2.h"
#include <stdint.h>

void InputManager_Init(void);
void InputManager_HandleExti(uint16_t gpio_pin);
void InputManager_Task(void);

#endif
