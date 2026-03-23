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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IR_ADC_LEFT_Pin GPIO_PIN_0
#define IR_ADC_LEFT_GPIO_Port GPIOA
#define IR_ADC_CENTER_Pin GPIO_PIN_1
#define IR_ADC_CENTER_GPIO_Port GPIOA
#define LiDAR_TX_Pin GPIO_PIN_2
#define LiDAR_TX_GPIO_Port GPIOA
#define LiDAR_RX_Pin GPIO_PIN_3
#define LiDAR_RX_GPIO_Port GPIOA
#define IR_ADC_RIGHT_Pin GPIO_PIN_4
#define IR_ADC_RIGHT_GPIO_Port GPIOA
#define US_TRIG_Pin GPIO_PIN_0
#define US_TRIG_GPIO_Port GPIOB
#define US_EXTI1_ECHO_LEFT_Pin GPIO_PIN_1
#define US_EXTI1_ECHO_LEFT_GPIO_Port GPIOB
#define US_EXTI2_ECHO_RIGHT_Pin GPIO_PIN_2
#define US_EXTI2_ECHO_RIGHT_GPIO_Port GPIOB
#define USART_ESP32_TX_Pin GPIO_PIN_15
#define USART_ESP32_TX_GPIO_Port GPIOA
#define USART_EXP32_RX_Pin GPIO_PIN_3
#define USART_EXP32_RX_GPIO_Port GPIOB
#define LiDAR_TIM2_PWM_Pin GPIO_PIN_5
#define LiDAR_TIM2_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
