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
#include "stm32l4xx_hal.h"

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
void App_ControlInit(void);
void App_UartRxCallback(UART_HandleTypeDef *huart);
void App_UartErrorCallback(UART_HandleTypeDef *huart);
void App_GpioExtiCallback(uint16_t GPIO_Pin);
void App_TimerElapsedCallback(TIM_HandleTypeDef *htim);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SAFE_Pin GPIO_PIN_1
#define SAFE_GPIO_Port GPIOB
#define SOS_Pin GPIO_PIN_2
#define SOS_GPIO_Port GPIOB
#define ENRD_Pin GPIO_PIN_7
#define ENRD_GPIO_Port GPIOC
#define EN_5V_Pin GPIO_PIN_5
#define EN_5V_GPIO_Port GPIOB
#define P_SWITCH_Pin GPIO_PIN_8
#define P_SWITCH_GPIO_Port GPIOB
#define POWER_KEEP_Pin GPIO_PIN_9
#define POWER_KEEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
