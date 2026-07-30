#ifndef K230_LINK_H
#define K230_LINK_H

#include "stm32f1xx_hal.h"

#include <stdint.h>

#define K230_LINK_RUNTIME_ENABLED 1U
#define K230_MAX_DETECTIONS       16U

typedef enum
{
  K230_TASK_NONE = 0x00,
  K230_TASK_NUMBER = 0x01,
  K230_TASK_BEAN = 0x02
} K230VisionTask;

typedef enum
{
  K230_TASK_SWITCH_IDLE = 0,
  K230_TASK_SWITCH_PENDING,
  K230_TASK_SWITCH_FAILED
} K230TaskSwitchState;

typedef enum
{
  K230_SEMANTIC_UNKNOWN = 0xFF,
  /* STM32内部继续使用紧凑语义值；串口线上接收的是ASCII '1'~'5'。 */
  K230_SEMANTIC_NUMBER_1 = 0x01,
  K230_SEMANTIC_NUMBER_2 = 0x02,
  K230_SEMANTIC_NUMBER_3 = 0x03,
  K230_SEMANTIC_NUMBER_4 = 0x04,
  K230_SEMANTIC_NUMBER_5 = 0x05,
  K230_SEMANTIC_BEAN_L = 0x11,
  K230_SEMANTIC_BEAN_H = 0x12,
  K230_SEMANTIC_BEAN_B = 0x13
} K230SemanticCode;

typedef struct
{
  uint8_t semantic;
  /* 简化字符协议不携带以下字段；接收层填入兼容占位值100%、(320,240)。 */
  uint8_t confidence_percent;
  uint16_t center_x;
  uint16_t center_y;
} K230VisionTarget;

/* 机器人规划层读取的完整视觉快照。 */
typedef struct
{
  uint8_t valid;
  uint8_t task;
  uint8_t count;
  uint8_t truncated;
  uint16_t sequence;
  uint32_t received_tick_ms;
  K230VisionTarget targets[K230_MAX_DETECTIONS];
} K230VisionResult;

typedef struct
{
  uint32_t valid_frames;
  /* 简化字符协议无CRC；保留字段以兼容现有诊断页，值恒为0。 */
  uint32_t crc_errors;
  uint32_t format_errors;
  uint32_t rx_overflows;
  uint32_t uart_errors;
  /* 保留原字段名；当前统计N/B模型切换确认超时。 */
  uint32_t request_timeouts;
} K230LinkStats;

/* 兼容旧上层接口：返回当前快照中置信度最高的目标。 */
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

/* 异步切换模型：发送'N'选数字、'B'选豆子，等待'n'/'b'确认。 */
uint8_t K230Link_SelectTask(K230VisionTask task);
K230VisionTask K230Link_GetSelectedTask(void);
K230VisionTask K230Link_GetRequestedTask(void);
K230TaskSwitchState K230Link_GetTaskSwitchState(void);

uint8_t K230Link_GetLatestResult(K230VisionResult *result);
uint8_t K230Link_IsResultFresh(uint32_t maximum_age_ms);
void K230Link_InvalidateResult(void);
void K230Link_GetStats(K230LinkStats *stats);

uint8_t K230Link_GetLatestDetection(K230Detection *detection);

#endif
