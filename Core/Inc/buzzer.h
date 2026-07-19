#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Beep(uint32_t duration_ms);
void Buzzer_Process(void);

#endif
