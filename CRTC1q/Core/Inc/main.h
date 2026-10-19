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
#define NFAULT1_Pin GPIO_PIN_0
#define NFAULT1_GPIO_Port GPIOA
#define NFAULT2_Pin GPIO_PIN_1
#define NFAULT2_GPIO_Port GPIOA
#define CH7800_TX_Pin GPIO_PIN_2
#define CH7800_TX_GPIO_Port GPIOA
#define CH7800_RX_Pin GPIO_PIN_3
#define CH7800_RX_GPIO_Port GPIOA
#define SCL_PCA9685_Pin GPIO_PIN_10
#define SCL_PCA9685_GPIO_Port GPIOB
#define SDA_PCA9685_Pin GPIO_PIN_11
#define SDA_PCA9685_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_8
#define SPI2_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
