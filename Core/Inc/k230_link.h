#ifndef K230_LINK_H
#define K230_LINK_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct
{
  uint8_t valid;
  uint8_t class_id;
  uint16_t confidence_permille;
  int16_t center_x;
  int16_t center_y;
  uint32_t frame_id;
} K230Detection;

void K230Link_Init(UART_HandleTypeDef *uart);
void K230Link_HandleRxEvent(UART_HandleTypeDef *uart, uint16_t size);
void K230Link_HandleTxComplete(UART_HandleTypeDef *uart);
void K230Link_HandleError(UART_HandleTypeDef *uart);
void K230Link_Task(void);
void K230Link_SendText(const char *text);
uint8_t K230Link_GetLatestDetection(K230Detection *detection);

#endif
