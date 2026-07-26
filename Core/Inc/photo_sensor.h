#ifndef PHOTO_SENSOR_H
#define PHOTO_SENSOR_H

#include <stdint.h>

#define PHOTO_SENSOR_COUNT 3U

void PhotoSensor_Init(void);
void PhotoSensor_HandleExti(uint16_t gpio_pin);
/* 返回GPIO原始电平：1为高电平，0为低电平；索引范围为0~2。 */
uint8_t PhotoSensor_GetState(uint8_t index);
/* 每一位对应一路发生过边沿变化，读取后清除。 */
uint8_t PhotoSensor_ConsumeChangedMask(void);

#endif
