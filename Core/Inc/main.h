/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AMPENA_Pin GPIO_PIN_0
#define AMPENA_GPIO_Port GPIOA
#define LEDTX_Pin GPIO_PIN_3
#define LEDTX_GPIO_Port GPIOA
#define LEDERR_Pin GPIO_PIN_4
#define LEDERR_GPIO_Port GPIOA
#define LEDRX_Pin GPIO_PIN_5
#define LEDRX_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOB
#define BTEN_Pin GPIO_PIN_11
#define BTEN_GPIO_Port GPIOB
#define AMPEN_Pin GPIO_PIN_12
#define AMPEN_GPIO_Port GPIOB
#define VOL__Pin GPIO_PIN_13
#define VOL__GPIO_Port GPIOB
#define VOL_B14_Pin GPIO_PIN_14
#define VOL_B14_GPIO_Port GPIOB
#define NEXT_Pin GPIO_PIN_15
#define NEXT_GPIO_Port GPIOB
#define PREV_Pin GPIO_PIN_8
#define PREV_GPIO_Port GPIOA
#define PLAY_Pin GPIO_PIN_9
#define PLAY_GPIO_Port GPIOA
#define CANRS_Pin GPIO_PIN_10
#define CANRS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
