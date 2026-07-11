#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
/* 在 EXTI 回调中调用，根据 B 相电平判断 A 相边沿方向。 */
void Encoder_HandleExti(uint16_t gpio_pin);
/* 获取累计计数或本次控制周期的增量。 */
int32_t Encoder_GetCount(uint8_t index);
int32_t Encoder_GetDelta(uint8_t index);

#endif
