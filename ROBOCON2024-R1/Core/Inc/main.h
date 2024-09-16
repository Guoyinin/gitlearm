/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define RX_NRF24L01_CHIP_SELECT_Pin GPIO_PIN_4
#define RX_NRF24L01_CHIP_SELECT_GPIO_Port GPIOA
#define TX_NRF24L01_CHIP_IQR_Pin GPIO_PIN_4
#define TX_NRF24L01_CHIP_IQR_GPIO_Port GPIOC
#define TX_NRF24L01_CHIP_ENABLE_Pin GPIO_PIN_5
#define TX_NRF24L01_CHIP_ENABLE_GPIO_Port GPIOC
#define RX_NRF24L01_CHIP_IQR_Pin GPIO_PIN_0
#define RX_NRF24L01_CHIP_IQR_GPIO_Port GPIOB
#define RX_NRF24L01_CHIP_ENABLE_Pin GPIO_PIN_1
#define RX_NRF24L01_CHIP_ENABLE_GPIO_Port GPIOB
#define TX_NRF24L01_CHIP_SELECT_Pin GPIO_PIN_12
#define TX_NRF24L01_CHIP_SELECT_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define RS485_EN_Pin GPIO_PIN_8
#define RS485_EN_GPIO_Port GPIOA
#define EBYTE_M0_Pin GPIO_PIN_4
#define EBYTE_M0_GPIO_Port GPIOD
#define EBYTE_M1_Pin GPIO_PIN_5
#define EBYTE_M1_GPIO_Port GPIOD
#define ARM_RIGHT_STRETCH_Pin GPIO_PIN_6
#define ARM_RIGHT_STRETCH_GPIO_Port GPIOD
#define ARM_RIGHT_RISE_Pin GPIO_PIN_7
#define ARM_RIGHT_RISE_GPIO_Port GPIOD
#define ARM_LEFT_STRETCH_Pin GPIO_PIN_3
#define ARM_LEFT_STRETCH_GPIO_Port GPIOB
#define ARM_LEFT_RISE_Pin GPIO_PIN_4
#define ARM_LEFT_RISE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
