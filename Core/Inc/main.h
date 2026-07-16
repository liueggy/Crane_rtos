/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIMIT_Z_MAX_Pin GPIO_PIN_2
#define LIMIT_Z_MAX_GPIO_Port GPIOE
#define LIMIT_Z_MAX_EXTI_IRQn EXTI2_IRQn
#define KEY_DIR_TOGGLE_Pin GPIO_PIN_3
#define KEY_DIR_TOGGLE_GPIO_Port GPIOE
#define KEY_DIR_TOGGLE_EXTI_IRQn EXTI3_IRQn
#define KEY_RUN_STOP_Pin GPIO_PIN_4
#define KEY_RUN_STOP_GPIO_Port GPIOE
#define KEY_RUN_STOP_EXTI_IRQn EXTI4_IRQn
#define LED_STATUS_Pin GPIO_PIN_5
#define LED_STATUS_GPIO_Port GPIOE
#define DC_M1_UNUSED_1_Pin GPIO_PIN_0
#define DC_M1_UNUSED_1_GPIO_Port GPIOC
#define DC_M1_UNUSED_2_Pin GPIO_PIN_1
#define DC_M1_UNUSED_2_GPIO_Port GPIOC
#define DC_M2_UNUSED_1_Pin GPIO_PIN_2
#define DC_M2_UNUSED_1_GPIO_Port GPIOC
#define DC_M2_UNUSED_2_Pin GPIO_PIN_3
#define DC_M2_UNUSED_2_GPIO_Port GPIOC
#define SERVO_1_PWM_Pin GPIO_PIN_0
#define SERVO_1_PWM_GPIO_Port GPIOA
#define SERVO_2_PWM_Pin GPIO_PIN_1
#define SERVO_2_PWM_GPIO_Port GPIOA
#define K230_UART_TX_Pin GPIO_PIN_2
#define K230_UART_TX_GPIO_Port GPIOA
#define K230_UART_RX_Pin GPIO_PIN_3
#define K230_UART_RX_GPIO_Port GPIOA
#define DC_M1_IN1_PWM_Pin GPIO_PIN_6
#define DC_M1_IN1_PWM_GPIO_Port GPIOA
#define DC_M1_IN2_PWM_Pin GPIO_PIN_7
#define DC_M1_IN2_PWM_GPIO_Port GPIOA
#define DC_M3_UNUSED_1_Pin GPIO_PIN_4
#define DC_M3_UNUSED_1_GPIO_Port GPIOC
#define LIMIT_Z_MIN_Pin GPIO_PIN_5
#define LIMIT_Z_MIN_GPIO_Port GPIOC
#define LIMIT_Z_MIN_EXTI_IRQn EXTI9_5_IRQn
#define DC_M2_IN1_PWM_Pin GPIO_PIN_0
#define DC_M2_IN1_PWM_GPIO_Port GPIOB
#define DC_M2_IN2_PWM_Pin GPIO_PIN_1
#define DC_M2_IN2_PWM_GPIO_Port GPIOB
#define ESTOP_IN_Pin GPIO_PIN_7
#define ESTOP_IN_GPIO_Port GPIOE
#define ESTOP_IN_EXTI_IRQn EXTI9_5_IRQn
#define STEP_X_PUL_Pin GPIO_PIN_9
#define STEP_X_PUL_GPIO_Port GPIOE
#define STEP_Z_PUL_Pin GPIO_PIN_11
#define STEP_Z_PUL_GPIO_Port GPIOE
#define ENC_M1_A_Pin GPIO_PIN_10
#define ENC_M1_A_GPIO_Port GPIOD
#define ENC_M1_A_EXTI_IRQn EXTI15_10_IRQn
#define STEP_X_ENA_Pin GPIO_PIN_11
#define STEP_X_ENA_GPIO_Port GPIOD
#define STEP_Z_ENA_Pin GPIO_PIN_12
#define STEP_Z_ENA_GPIO_Port GPIOD
#define ENC_M2_A_Pin GPIO_PIN_13
#define ENC_M2_A_GPIO_Port GPIOD
#define ENC_M2_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC_M3_A_Pin GPIO_PIN_14
#define ENC_M3_A_GPIO_Port GPIOD
#define ENC_M3_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC_M4_A_Pin GPIO_PIN_15
#define ENC_M4_A_GPIO_Port GPIOD
#define ENC_M4_A_EXTI_IRQn EXTI15_10_IRQn
#define DC_M3_IN1_PWM_Pin GPIO_PIN_6
#define DC_M3_IN1_PWM_GPIO_Port GPIOC
#define DC_M3_IN2_PWM_Pin GPIO_PIN_7
#define DC_M3_IN2_PWM_GPIO_Port GPIOC
#define DC_M4_IN1_PWM_Pin GPIO_PIN_8
#define DC_M4_IN1_PWM_GPIO_Port GPIOC
#define DC_M4_IN2_PWM_Pin GPIO_PIN_9
#define DC_M4_IN2_PWM_GPIO_Port GPIOC
#define STEP_X_DIR_Pin GPIO_PIN_2
#define STEP_X_DIR_GPIO_Port GPIOD
#define STEP_Z_DIR_Pin GPIO_PIN_3
#define STEP_Z_DIR_GPIO_Port GPIOD
#define ENC_M1_B_Pin GPIO_PIN_4
#define ENC_M1_B_GPIO_Port GPIOD
#define ENC_M2_B_Pin GPIO_PIN_5
#define ENC_M2_B_GPIO_Port GPIOD
#define ENC_M3_B_Pin GPIO_PIN_6
#define ENC_M3_B_GPIO_Port GPIOD
#define ENC_M4_B_Pin GPIO_PIN_7
#define ENC_M4_B_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOB
#define IR_REMOTE_RX_Pin GPIO_PIN_9
#define IR_REMOTE_RX_GPIO_Port GPIOB
#define LIMIT_X_MIN_Pin GPIO_PIN_0
#define LIMIT_X_MIN_GPIO_Port GPIOE
#define LIMIT_X_MIN_EXTI_IRQn EXTI0_IRQn
#define LIMIT_X_MAX_Pin GPIO_PIN_1
#define LIMIT_X_MAX_GPIO_Port GPIOE
#define LIMIT_X_MAX_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
