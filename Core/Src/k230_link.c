#include "k230_link.h"

#include "app_state.h"
#include "cmsis_os2.h"
#include "robot_controller.h"
#include "servo_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define K230_RX_BUFFER_SIZE 128U
#define K230_TX_BUFFER_SIZE 192U
#define K230_EVENT_RX_FRAME (1UL << 0)

static UART_HandleTypeDef *g_uart;
static osEventFlagsId_t g_events;
static uint8_t g_rx_dma[K230_RX_BUFFER_SIZE];
static char g_command[K230_RX_BUFFER_SIZE];
static uint16_t g_command_length;
static uint8_t g_tx_buffer[K230_TX_BUFFER_SIZE];
static volatile uint8_t g_tx_busy;
static K230Detection g_detection;

static void StartReceive(void)
{
  if ((g_uart == NULL) || (g_uart->hdmarx == NULL)) return;
  __HAL_DMA_DISABLE_IT(g_uart->hdmarx, DMA_IT_HT);
  (void)HAL_UARTEx_ReceiveToIdle_DMA(g_uart, g_rx_dma, sizeof(g_rx_dma));
}

void K230Link_Init(UART_HandleTypeDef *uart)
{
  g_uart = uart;
  g_events = osEventFlagsNew(0);
  memset(&g_detection, 0, sizeof(g_detection));
  StartReceive();
}

void K230Link_SendText(const char *text)
{
  size_t length;
  if ((g_uart == NULL) || (text == NULL)) return;
  length = strlen(text);
  if (length >= sizeof(g_tx_buffer)) length = sizeof(g_tx_buffer) - 1U;
  while (g_tx_busy) osDelay(1U);
  memcpy(g_tx_buffer, text, length);
  g_tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(g_uart, g_tx_buffer, (uint16_t)length) != HAL_OK) g_tx_busy = 0U;
}

static uint8_t ParseLong(const char *text, long *value)
{
  char *end;
  while ((*text == ' ') || (*text == '=')) ++text;
  *value = strtol(text, &end, 10);
  return (end != text) && (*end == '\0');
}

static void ProcessCommand(char *command)
{
  long value;
  char response[80];
  while ((*command == ' ') || (*command == '\t')) ++command;
  if (strcmp(command, "PING") == 0)
  {
    AppState_SetK230Online(1U);
    K230Link_SendText("PONG\r\n");
  }
  else if (strcmp(command, "AUTO START") == 0)
  {
    RobotController_RequestStart();
    K230Link_SendText("AUTO ARMED\r\n");
  }
  else if (strcmp(command, "AUTO ABORT") == 0)
  {
    RobotController_RequestAbort();
    K230Link_SendText("AUTO ABORTED\r\n");
  }
  else if ((command[0] == 'A') && ((command[1] == '1') || (command[1] == '2')) &&
           ParseLong(command + 2, &value) && (value >= 0L) && (value <= 270L))
  {
    uint8_t index = (uint8_t)(command[1] - '1');
    ServoControl_SetAngle(index, (uint16_t)value);
    (void)snprintf(response, sizeof(response), "OK A%u=%ld\r\n", (unsigned)(index + 1U), value);
    K230Link_SendText(response);
  }
  else if ((command[0] == 'S') && ((command[1] == '1') || (command[1] == '2')) &&
           ParseLong(command + 2, &value) && (value >= 0L) && (value <= 2500L))
  {
    uint8_t index = (uint8_t)(command[1] - '1');
    ServoControl_SetPulseUs(index, (uint16_t)value);
    (void)snprintf(response, sizeof(response), "OK S%u=%ld\r\n", (unsigned)(index + 1U), value);
    K230Link_SendText(response);
  }
  else if (strcmp(command, "GET") == 0)
  {
    (void)snprintf(response, sizeof(response), "S1=%u S2=%u\r\n",
                   (unsigned)ServoControl_GetPulseUs(0U), (unsigned)ServoControl_GetPulseUs(1U));
    K230Link_SendText(response);
  }
  else K230Link_SendText("ERR unknown command\r\n");
}

void K230Link_HandleRxEvent(UART_HandleTypeDef *uart, uint16_t size)
{
  if ((uart != g_uart) || (size == 0U)) return;
  if (size >= sizeof(g_command)) size = sizeof(g_command) - 1U;
  memcpy(g_command, g_rx_dma, size);
  g_command[size] = '\0';
  g_command_length = size;
  if (g_events != NULL) (void)osEventFlagsSet(g_events, K230_EVENT_RX_FRAME);
  StartReceive();
}

void K230Link_HandleTxComplete(UART_HandleTypeDef *uart)
{
  if (uart == g_uart) g_tx_busy = 0U;
}

void K230Link_HandleError(UART_HandleTypeDef *uart)
{
  if (uart == g_uart) StartReceive();
}

void K230Link_Task(void)
{
  char command[K230_RX_BUFFER_SIZE];
  K230Link_SendText("STM32 READY\r\n");
  for (;;)
  {
    uint32_t flags = osEventFlagsWait(g_events, K230_EVENT_RX_FRAME, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U) continue;
    memcpy(command, g_command, g_command_length);
    command[g_command_length] = '\0';
    for (char *cursor = command; *cursor != '\0'; ++cursor)
    {
      if ((*cursor == '\r') || (*cursor == '\n')) { *cursor = '\0'; break; }
    }
    ProcessCommand(command);
  }
}

uint8_t K230Link_GetLatestDetection(K230Detection *detection)
{
  if ((detection == NULL) || !g_detection.valid) return 0U;
  *detection = g_detection;
  return 1U;
}
