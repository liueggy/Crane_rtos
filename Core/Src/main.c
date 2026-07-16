/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "app_state.h"
#include "chassis_motion.h"
#include "dc_motor.h"
#include "encoder.h"
#include "input_manager.h"
#include "infrared_remote.h"
#include "k230_link.h"
#include "motor_control.h"
#include "robot_controller.h"
#include "safety_manager.h"
#include "servo_control.h"
#include "stepper_axis.h"
#include "ui_manager.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for APP_CTRL */
osThreadId_t APP_CTRLHandle;
const osThreadAttr_t APP_CTRL_attributes = {
  .name = "APP_CTRL",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DC_MOTOR */
osThreadId_t DC_MOTORHandle;
const osThreadAttr_t DC_MOTOR_attributes = {
  .name = "DC_MOTOR",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for STEPPER */
osThreadId_t STEPPERHandle;
const osThreadAttr_t STEPPER_attributes = {
  .name = "STEPPER",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for K230_RX */
osThreadId_t K230_RXHandle;
const osThreadAttr_t K230_RX_attributes = {
  .name = "K230_RX",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for INPUT_EVT */
osThreadId_t INPUT_EVTHandle;
const osThreadAttr_t INPUT_EVT_attributes = {
  .name = "INPUT_EVT",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for OLED */
osThreadId_t OLEDHandle;
const osThreadAttr_t OLED_attributes = {
  .name = "OLED",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
void StartAppCtrlTask(void *argument);
void StartDcMotorTask(void *argument);
void StartStepperTask(void *argument);
void StartK230RxTask(void *argument);
void StartInputEvtTask(void *argument);
void StartOledTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if 0 /* 旧版业务实现已迁移至 Core/Src 对应模块，保留一版便于本次结构迁移审阅。 */

static uint16_t Servo_ClampPulse(uint8_t index, uint16_t pulse_us)
{
  AppConfig config;
  AppConfig_GetSnapshot(&config);
  if (index >= 2U)
  {
    return 0U;
  }
  if (pulse_us < config.servo_min_us[index])
  {
    return config.servo_min_us[index];
  }
  if (pulse_us > config.servo_max_us[index])
  {
    return config.servo_max_us[index];
  }
  return pulse_us;
}

static void Servo_SetPulseUsByIndex(uint8_t index, uint16_t pulse_us)
{
  uint32_t channel;
  if (index >= 2U)
  {
    return;
  }
  pulse_us = Servo_ClampPulse(index, pulse_us);
  channel = (index == 0U) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
  __HAL_TIM_SET_COMPARE(&htim2, channel, pulse_us);
  g_servo_pulse_us[index] = pulse_us;
}

static void Servo_SetAngle(uint8_t index, uint16_t angle)
{
  AppConfig config;
  uint32_t pulse;
  AppConfig_GetSnapshot(&config);
  if (index >= 2U)
  {
    return;
  }
  if (angle > 270U)
  {
    angle = 270U;
  }
  pulse = config.servo_min_us[index] +
          ((uint32_t)(config.servo_max_us[index] - config.servo_min_us[index]) * angle) / 270U;
  Servo_SetPulseUsByIndex(index, (uint16_t)pulse);
  g_servo_angle[index] = angle;
}

static void Uart2_StartReceive(void)
{
  memset(g_uart2_rx_dma, 0, sizeof(g_uart2_rx_dma));
  if (huart2.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_uart2_rx_dma, sizeof(g_uart2_rx_dma));
}

static void Uart2_SendText(const char *text)
{
  size_t length = strlen(text);
  if (length >= sizeof(g_uart2_tx_buffer))
  {
    length = sizeof(g_uart2_tx_buffer) - 1U;
  }
  while (g_uart2_tx_busy)
  {
    osDelay(1U);
  }
  memcpy(g_uart2_tx_buffer, text, length);
  g_uart2_tx_buffer[length] = '\0';
  g_uart2_tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&huart2, g_uart2_tx_buffer, (uint16_t)length) != HAL_OK)
  {
    g_uart2_tx_busy = 0U;
  }
}

static void DcMotorTest_SelectNextGear(void)
{
  AppConfig config;
  AppState state;
  char response[UART2_TX_BUFFER_SIZE];

  AppConfig_GetSnapshot(&config);
  AppState_GetSnapshot(&state);
  if (state.estop_active)
  {
    Uart2_SendText("DC TEST BLOCKED: ESTOP\r\n");
    return;
  }

  if ((g_dc_test_pwm + config.motor_test_pwm_step) > config.motor_test_pwm_limit)
  {
    g_dc_test_pwm = 0U;
  }
  else
  {
    g_dc_test_pwm += config.motor_test_pwm_step;
  }
  AppState_SetRunEnabled(g_dc_test_pwm > 0U);
  AppState_SetDcTestState((uint8_t)((g_dc_test_pwm + config.motor_test_pwm_step - 1U) /
                                    config.motor_test_pwm_step),
                          g_dc_test_direction_reverse, g_dc_test_pwm);
  (void)snprintf(response, sizeof(response),
                 "DC M1 GEAR=%u PWM=%u DIR=%s\r\n",
                 (unsigned)((g_dc_test_pwm + config.motor_test_pwm_step - 1U) /
                            config.motor_test_pwm_step),
                 (unsigned)g_dc_test_pwm,
                 g_dc_test_direction_reverse ? "REV" : "FWD");
  Uart2_SendText(response);
}

static void DcMotorTest_ToggleDirection(void)
{
  AppConfig config;
  char response[UART2_TX_BUFFER_SIZE];

  AppConfig_GetSnapshot(&config);
  g_dc_test_direction_reverse ^= 1U;
  AppState_SetDcTestState((uint8_t)((g_dc_test_pwm + config.motor_test_pwm_step - 1U) /
                                    config.motor_test_pwm_step),
                          g_dc_test_direction_reverse, g_dc_test_pwm);
  (void)snprintf(response, sizeof(response), "DC M1 DIR=%s\r\n",
                 g_dc_test_direction_reverse ? "REV" : "FWD");
  Uart2_SendText(response);
}

static void DcPid_AdjustTarget(int16_t delta_rpm)
{
  AppConfig config;
  char response[UART2_TX_BUFFER_SIZE];
  int32_t target;

  AppConfig_GetSnapshot(&config);
  target = (int32_t)g_dc_target_rpm + delta_rpm;
  if (target > (int32_t)config.maximum_rpm)
  {
    target = (int32_t)config.maximum_rpm;
  }
  else if (target < -(int32_t)config.maximum_rpm)
  {
    target = -(int32_t)config.maximum_rpm;
  }
  g_dc_target_rpm = (int16_t)target;
  (void)snprintf(response, sizeof(response), "PID TARGET=%d RPM\r\n",
                 (int)g_dc_target_rpm);
  Uart2_SendText(response);
}

static uint8_t Uart2_ParseValue(const char *text, long *value)
{
  char *end;
  while ((*text == ' ') || (*text == '='))
  {
    ++text;
  }
  *value = strtol(text, &end, 10);
  return (end != text) && ((*end == '\0') || (*end == '\r') || (*end == '\n'));
}

static void Uart2_ProcessCommand(char *command)
{
  char response[UART2_TX_BUFFER_SIZE];
  long value;
  uint8_t index;

  while ((*command == ' ') || (*command == '\t'))
  {
    ++command;
  }
  for (char *cursor = command; *cursor != '\0'; ++cursor)
  {
    if ((*cursor == '\r') || (*cursor == '\n'))
    {
      *cursor = '\0';
      break;
    }
  }

  if (strcmp(command, "PING") == 0)
  {
    Uart2_SendText("PONG\r\n");
    return;
  }
  if (strcmp(command, "HELP") == 0)
  {
    Uart2_SendText("PING | GET | S1/S2 <us> | A1/A2 <deg 0..270> | ALL <us> | ALLA <deg>\r\n");
    return;
  }
  if (strcmp(command, "GET") == 0)
  {
    (void)snprintf(response, sizeof(response), "S1=%u S2=%u\r\n",
                   (unsigned)g_servo_pulse_us[0], (unsigned)g_servo_pulse_us[1]);
    Uart2_SendText(response);
    return;
  }
  if (strncmp(command, "ALLA", 4) == 0)
  {
    if (Uart2_ParseValue(command + 4, &value) && (value >= 0L) && (value <= 270L))
    {
      Servo_SetAngle(0U, (uint16_t)value);
      Servo_SetAngle(1U, (uint16_t)value);
      (void)snprintf(response, sizeof(response), "OK ALLA=%ld\r\n", value);
    }
    else
    {
      (void)snprintf(response, sizeof(response), "ERR angle 0..270\r\n");
    }
    Uart2_SendText(response);
    return;
  }
  if (strncmp(command, "ALL", 3) == 0)
  {
    if (Uart2_ParseValue(command + 3, &value) && (value >= 0L) && (value <= 2500L))
    {
      Servo_SetPulseUsByIndex(0U, (uint16_t)value);
      Servo_SetPulseUsByIndex(1U, (uint16_t)value);
      (void)snprintf(response, sizeof(response), "OK ALL=%u\r\n",
                     (unsigned)g_servo_pulse_us[0]);
    }
    else
    {
      (void)snprintf(response, sizeof(response), "ERR pulse\r\n");
    }
    Uart2_SendText(response);
    return;
  }
  if ((command[0] == 'S') || (command[0] == 'A'))
  {
    if ((command[1] == '1') || (command[1] == '2'))
    {
      index = (uint8_t)(command[1] - '1');
      if (Uart2_ParseValue(command + 2, &value) &&
           ((command[0] == 'A' && value >= 0L && value <= 270L) ||
           (command[0] == 'S' && value >= 0L && value <= 2500L)))
      {
        if (command[0] == 'A')
        {
          Servo_SetAngle(index, (uint16_t)value);
        }
        else
        {
          Servo_SetPulseUsByIndex(index, (uint16_t)value);
        }
        (void)snprintf(response, sizeof(response), "OK %c%u=%u\r\n", command[0],
                       (unsigned)(index + 1U), (unsigned)g_servo_pulse_us[index]);
      }
      else
      {
        (void)snprintf(response, sizeof(response), "ERR value\r\n");
      }
      Uart2_SendText(response);
      return;
    }
  }
  Uart2_SendText("ERR unknown command, send HELP\r\n");
}

static void StepMotor_SetEnabled(uint8_t enabled)
{
  /* TB6600 的 ENA 为低电平有效，逻辑层使用 enabled=1 表示使能。 */
  HAL_GPIO_WritePin(STEP_Z_ENA_GPIO_Port, STEP_Z_ENA_Pin, enabled ? MOTOR_ENABLE_STATE : MOTOR_DISABLE_STATE);
}

static void StepMotor_ApplyRunState(void)
{
  /* 同时控制驱动器使能和脉冲占空比，停止时不继续发步进脉冲。 */
  StepMotor_SetEnabled(g_motor_run);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, g_motor_run ? STEP_MOTOR_RUN_PULSE : STEP_MOTOR_STOP_PULSE);
  AppState_SetStepperTelemetry(g_motor_run,
                               HAL_GPIO_ReadPin(STEP_Z_DIR_GPIO_Port, STEP_Z_DIR_Pin) == GPIO_PIN_SET,
                               g_motor_run ? STEP_MOTOR_RUN_PULSE : STEP_MOTOR_STOP_PULSE);
}

static void Led_ApplyRunState(void)
{
  if (g_motor_run)
  {
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);  /* 绿灯亮 */
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);          /* 红灯灭 */
  }
  else
  {
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);    /* 绿灯灭 */
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);        /* 红灯亮 */
  }
}

static void Motor_ToggleRunState(void)
{
  AppState state;
  AppState_GetSnapshot(&state);
  if (state.estop_active)
  {
    return;
  }
  g_motor_run = !g_motor_run;
  AppState_SetRunEnabled(g_motor_run);
  StepMotor_ApplyRunState();
  Led_ApplyRunState();
}

static uint8_t Key_IsPressed(GPIO_TypeDef *port, uint16_t pin)
{
  return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET;
}

static uint8_t Key0_IsPressed(void)
{
  return Key_IsPressed(KEY_RUN_STOP_GPIO_Port, KEY_RUN_STOP_Pin);
}

static uint8_t Key1_IsPressed(void)
{
  return Key_IsPressed(KEY_DIR_TOGGLE_GPIO_Port, KEY_DIR_TOGGLE_Pin);
}

static void Motor_ToggleDirection(void)
{
  /* TB6600 要求 DIR 在脉冲间保持稳定，换向前先暂停当前脉冲。 */
  if (g_motor_run)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, STEP_MOTOR_STOP_PULSE);
    osDelay(2U);
  }
  HAL_GPIO_TogglePin(STEP_Z_DIR_GPIO_Port, STEP_Z_DIR_Pin);
  StepMotor_ApplyRunState();
}

#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  AppConfig_Init();
  AppState_Init();
  Encoder_Init();
  SafetyManager_Init();
  ServoControl_Init(&htim2);
  StepperAxis_Init(&htim1);
  ChassisMotion_Init();
  RobotController_Init();
  if (InfraredRemote_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  AppConfig config;
  AppConfig_GetSnapshot(&config);
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  ServoControl_SetAngle(0U, 90U);
  ServoControl_SetPulseUs(1U, config.servo_min_us[1]);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  StepperAxis_StopAll();

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0U);
  DcMotor_Init(&htim3, &htim8);
  MotorControl_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of APP_CTRL */
  APP_CTRLHandle = osThreadNew(StartAppCtrlTask, NULL, &APP_CTRL_attributes);

  /* creation of DC_MOTOR */
  DC_MOTORHandle = osThreadNew(StartDcMotorTask, NULL, &DC_MOTOR_attributes);

  /* creation of STEPPER */
  STEPPERHandle = osThreadNew(StartStepperTask, NULL, &STEPPER_attributes);

  /* creation of K230_RX */
  K230_RXHandle = osThreadNew(StartK230RxTask, NULL, &K230_RX_attributes);

  /* creation of INPUT_EVT */
  INPUT_EVTHandle = osThreadNew(StartInputEvtTask, NULL, &INPUT_EVT_attributes);

  /* creation of OLED */
  OLEDHandle = osThreadNew(StartOledTask, NULL, &OLED_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  InputManager_Init();
  K230Link_Init(&huart2);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999 ;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* TIM4 是通用定时器，不支持 BOTHEDGE 硬件捕获。
   * 初始化为下降沿捕获（9ms 引导码起始沿），
   * 由 InfraredRemote_HandleCapture 每次捕获后翻转极性。 */
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 71;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 99;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DC_M1_UNUSED_1_Pin|DC_M1_UNUSED_2_Pin|DC_M2_UNUSED_1_Pin|DC_M2_UNUSED_2_Pin
                          |DC_M3_UNUSED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, STEP_X_ENA_Pin|STEP_Z_ENA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, STEP_X_DIR_Pin|STEP_Z_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LIMIT_Z_MAX_Pin LIMIT_X_MIN_Pin LIMIT_X_MAX_Pin */
  GPIO_InitStruct.Pin = LIMIT_Z_MAX_Pin|LIMIT_X_MIN_Pin|LIMIT_X_MAX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_DIR_TOGGLE_Pin KEY_RUN_STOP_Pin ESTOP_IN_Pin */
  GPIO_InitStruct.Pin = KEY_DIR_TOGGLE_Pin|KEY_RUN_STOP_Pin|ESTOP_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_M1_UNUSED_1_Pin DC_M1_UNUSED_2_Pin DC_M2_UNUSED_1_Pin DC_M2_UNUSED_2_Pin
                           DC_M3_UNUSED_1_Pin */
  GPIO_InitStruct.Pin = DC_M1_UNUSED_1_Pin|DC_M1_UNUSED_2_Pin|DC_M2_UNUSED_1_Pin|DC_M2_UNUSED_2_Pin
                          |DC_M3_UNUSED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LIMIT_Z_MIN_Pin */
  GPIO_InitStruct.Pin = LIMIT_Z_MIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LIMIT_Z_MIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_M1_A_Pin ENC_M2_A_Pin ENC_M3_A_Pin ENC_M4_A_Pin */
  GPIO_InitStruct.Pin = ENC_M1_A_Pin|ENC_M2_A_Pin|ENC_M3_A_Pin|ENC_M4_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_X_ENA_Pin STEP_X_DIR_Pin STEP_Z_DIR_Pin */
  GPIO_InitStruct.Pin = STEP_X_ENA_Pin|STEP_X_DIR_Pin|STEP_Z_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP_Z_ENA_Pin */
  GPIO_InitStruct.Pin = STEP_Z_ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEP_Z_ENA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_M1_B_Pin ENC_M2_B_Pin ENC_M3_B_Pin ENC_M4_B_Pin */
  GPIO_InitStruct.Pin = ENC_M1_B_Pin|ENC_M2_B_Pin|ENC_M3_B_Pin|ENC_M4_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BEEP_Pin */
  GPIO_InitStruct.Pin = BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == ENC_M1_A_Pin) || (GPIO_Pin == ENC_M2_A_Pin) ||
      (GPIO_Pin == ENC_M3_A_Pin) || (GPIO_Pin == ENC_M4_A_Pin))
  {
    Encoder_HandleExti(GPIO_Pin);
  }
  else if (GPIO_Pin == ESTOP_IN_Pin)
  {
    SafetyManager_TriggerEstop();
    InputManager_HandleExti(GPIO_Pin);
  }
  else
  {
    SafetyManager_HandleExti(GPIO_Pin);
    InputManager_HandleExti(GPIO_Pin);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  K230Link_HandleRxEvent(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  K230Link_HandleTxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  K230Link_HandleError(huart);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  InfraredRemote_HandleCapture(htim);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartAppCtrlTask */
/**
  * @brief  Application state/control task.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAppCtrlTask */
void StartAppCtrlTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  for(;;)
  {
    InfraredRemote_Process();
    RobotController_Update();
    osDelay(10U);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDcMotorTask */
/**
  * @brief  Four-wheel DC motor control task.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDcMotorTask */
void StartDcMotorTask(void *argument)
{
  /* USER CODE BEGIN StartDcMotorTask */
  for (;;)
  {
    AppConfig config;
    AppConfig_GetSnapshot(&config);
    ChassisMotion_TaskStep();
    osDelay(config.motor_control_period_ms);
  }
  /* USER CODE END StartDcMotorTask */
}

/* USER CODE BEGIN Header_StartStepperTask */
/**
  * @brief  Stepper axis control task.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStepperTask */
void StartStepperTask(void *argument)
{
  /* USER CODE BEGIN StartStepperTask */
  for (;;)
  {
    StepperAxis_UpdateTelemetry();
    osDelay(20);
  }
  /* USER CODE END StartStepperTask */
}

/* USER CODE BEGIN Header_StartK230RxTask */
/**
  * @brief  K230 UART receive/parse task.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartK230RxTask */
void StartK230RxTask(void *argument)
{
  /* USER CODE BEGIN StartK230RxTask */
  K230Link_Task();
  /* USER CODE END StartK230RxTask */
}

/* USER CODE BEGIN Header_StartInputEvtTask */
/**
  * @brief  Button, limit, and safety input event task.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInputEvtTask */
void StartInputEvtTask(void *argument)
{
  /* USER CODE BEGIN StartInputEvtTask */
  InputManager_Task();
  /* USER CODE END StartInputEvtTask */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
  * @brief  OLED status display task.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument)
{
  /* USER CODE BEGIN StartOledTask */
  AppConfig config;

  osDelay(20);
  UiManager_Init(&hi2c1);

  for (;;)
  {
    /* 显示任务只负责刷新 UI，不直接驱动舵机或电机。 */
    AppConfig_GetSnapshot(&config);
    UiManager_Render();
    osDelay(config.ui_refresh_period_ms);
  }
  /* USER CODE END StartOledTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
