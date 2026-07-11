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
#include "dc_motor.h"
#include "encoder.h"
#include "motor_control.h"
#include "ui_manager.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_MOTOR_RUN_PULSE 500U
#define STEP_MOTOR_STOP_PULSE 0U
#define KEY_POLL_IDLE_MS 100U
#define MOTOR_ENABLE_STATE GPIO_PIN_RESET
#define MOTOR_DISABLE_STATE GPIO_PIN_SET
#define INPUT_FLAG_KEY0 (1UL << 0)
#define INPUT_FLAG_KEY1 (1UL << 1)
#define INPUT_FLAG_ESTOP (1UL << 2)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

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
volatile uint8_t g_motor_run = 0;  // 0=停止, 1=运行
static osEventFlagsId_t g_input_events;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
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

static void Servo_SetPulseUs(uint16_t pulse_us)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
}

static void StepMotor_SetEnabled(uint8_t enabled)
{
  /* TB6600 的 ENA 为低电平有效，逻辑层使用 enabled=1 表示使能。 */
  HAL_GPIO_WritePin(STEP_X_ENA_GPIO_Port, STEP_X_ENA_Pin, enabled ? MOTOR_ENABLE_STATE : MOTOR_DISABLE_STATE);
}

static void StepMotor_ApplyRunState(void)
{
  /* 同时控制驱动器使能和脉冲占空比，停止时不继续发步进脉冲。 */
  StepMotor_SetEnabled(g_motor_run);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, g_motor_run ? STEP_MOTOR_RUN_PULSE : STEP_MOTOR_STOP_PULSE);
}

static void Led_ApplyRunState(void)
{
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, g_motor_run ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  HAL_GPIO_TogglePin(STEP_X_DIR_GPIO_Port, STEP_X_DIR_Pin);
}

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  AppConfig_Init();
  AppState_Init();
  Encoder_Init();

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
  Servo_SetPulseUs(config.servo_min_us[0]);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, config.servo_min_us[1]);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  StepMotor_ApplyRunState();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, STEP_MOTOR_STOP_PULSE);

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
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  DcMotor_Init(&htim3);
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
  g_input_events = osEventFlagsNew(NULL);
  if (g_input_events == NULL)
  {
    Error_Handler();
  }
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
  htim1.Init.Period = 999;
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
  HAL_GPIO_WritePin(GPIOC, DC_M1_IN1_Pin|DC_M1_IN2_Pin|DC_M2_IN1_Pin|DC_M2_IN2_Pin
                          |DC_M3_IN1_Pin|DC_M3_IN2_Pin|DC_M4_IN2_Pin|DC_M4_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, STEP_X_ENA_Pin|STEP_Z_ENA_Pin|STEP_X_DIR_Pin|STEP_Z_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|BEEP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LIMIT_Z_MAX_Pin KEY_DIR_TOGGLE_Pin KEY_RUN_STOP_Pin ESTOP_IN_Pin
                           LIMIT_X_MIN_Pin LIMIT_X_MAX_Pin */
  GPIO_InitStruct.Pin = LIMIT_Z_MAX_Pin|KEY_DIR_TOGGLE_Pin|KEY_RUN_STOP_Pin|ESTOP_IN_Pin
                          |LIMIT_X_MIN_Pin|LIMIT_X_MAX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_M1_IN1_Pin DC_M1_IN2_Pin DC_M2_IN1_Pin DC_M2_IN2_Pin
                           DC_M3_IN1_Pin DC_M3_IN2_Pin DC_M4_IN2_Pin DC_M4_IN1_Pin */
  GPIO_InitStruct.Pin = DC_M1_IN1_Pin|DC_M1_IN2_Pin|DC_M2_IN1_Pin|DC_M2_IN2_Pin
                          |DC_M3_IN1_Pin|DC_M3_IN2_Pin|DC_M4_IN2_Pin|DC_M4_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pins : LED_RED_Pin BEEP_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  /* 中断只记录事件，消抖和业务处理放到 INPUT_EVT 任务中完成。 */
  if (GPIO_Pin == KEY_RUN_STOP_Pin)
  {
    if (g_input_events != NULL) (void)osEventFlagsSet(g_input_events, INPUT_FLAG_KEY0);
  }
  else if (GPIO_Pin == KEY_DIR_TOGGLE_Pin)
  {
    if (g_input_events != NULL) (void)osEventFlagsSet(g_input_events, INPUT_FLAG_KEY1);
  }
  else if (GPIO_Pin == ESTOP_IN_Pin)
  {
    if (g_input_events != NULL) (void)osEventFlagsSet(g_input_events, INPUT_FLAG_ESTOP);
  }
  else if ((GPIO_Pin == ENC_M1_A_Pin) || (GPIO_Pin == ENC_M2_A_Pin) ||
           (GPIO_Pin == ENC_M3_A_Pin) || (GPIO_Pin == ENC_M4_A_Pin))
  {
    Encoder_HandleExti(GPIO_Pin);
  }
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
  /* USER CODE BEGIN StartAppCtrlTask */
  for (;;)
  {
    /* 应用主控任务负责同步整机状态；自动状态机将在此接入。 */
    Led_ApplyRunState();
    osDelay(50);
  }
  /* USER CODE END StartAppCtrlTask */
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
  AppConfig config;
  for (;;)
  {
    /* 固定周期执行四轮测速和 PI 闭环输出。 */
    AppConfig_GetSnapshot(&config);
    MotorControl_Update(AppState_GetRunEnabled());
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
    /* 步进轴任务统一处理 X/Z 轴脉冲、方向、使能和限位。 */
    StepMotor_ApplyRunState();
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
  for (;;)
  {
    /* K230 接收任务预留给视觉结果和串口调参协议解析。 */
    osDelay(20);
  }
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
  AppConfig config;
  for (;;)
  {
    /* 按键事件在此消抖；限位和急停也在此汇总到 AppState。 */
    uint32_t flags = osEventFlagsWait(g_input_events,
                                      INPUT_FLAG_KEY0 | INPUT_FLAG_KEY1 | INPUT_FLAG_ESTOP,
                                      osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U)
    {
      continue;
    }
    AppConfig_GetSnapshot(&config);

    if ((flags & INPUT_FLAG_KEY0) != 0U)
    {
      osDelay(config.key_debounce_ms);

      if (Key0_IsPressed())
      {
        Motor_ToggleRunState();
        while (Key0_IsPressed())
        {
          osDelay(10);
        }
      }
    }

    if ((flags & INPUT_FLAG_KEY1) != 0U)
    {
      uint32_t pressed_ms = 0U;
      osDelay(config.key_debounce_ms);

      if (Key1_IsPressed())
      {
        while (Key1_IsPressed())
        {
          osDelay(10);
          pressed_ms += 10U;
        }
        if (pressed_ms >= config.key_long_press_ms) UiManager_NextPage();
        else Motor_ToggleDirection();
      }
    }

    if ((flags & INPUT_FLAG_ESTOP) != 0U)
    {
      g_motor_run = 0U;
      AppState_SetEstopActive(1U);
      MotorControl_Reset();
      StepMotor_ApplyRunState();
      Led_ApplyRunState();
    }
  }
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
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
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
