#ifndef PHOTO_SENSOR_H
#define PHOTO_SENSOR_H

#include <stdint.h>

#define PHOTO_SENSOR_COUNT 3U

typedef enum
{
  PHOTO_SENSOR_SHARED_XZ = 0,       /* PB11：X/Z轴共用挡片检测。 */
  PHOTO_SENSOR_CHASSIS_FAR_SIDE,    /* PB12：远离世界坐标原点的一侧。 */
  PHOTO_SENSOR_CHASSIS_ORIGIN_SIDE, /* PD8：靠近世界坐标原点的一侧。 */
} PhotoSensorId;

void PhotoSensor_Init(void);
void PhotoSensor_HandleExti(uint16_t gpio_pin);
/* 返回GPIO原始电平：1为遮挡，0为无遮挡。 */
uint8_t PhotoSensor_GetState(uint8_t index);
/* 每一位对应一路发生过边沿变化，读取后清除。 */
uint8_t PhotoSensor_ConsumeChangedMask(void);

#endif
