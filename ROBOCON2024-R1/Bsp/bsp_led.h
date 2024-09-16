/**
  ******************************************************************************
  * @file     bsp_led.h
  * @author   Junshuo
  * @version  V1.0
  * @date     Jun-28-2023
  * @brief    Epoch2023新主控板LED定义 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_LED_H
#define BSP_LED_H

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define LED1_OFF()    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET)
#define LED1_ON()     HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET)
#define	LED1_TOGGLE() HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin)

#define LED2_OFF()    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET)
#define LED2_ON()     HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET)
#define	LED2_TOGGLE() HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin)

#define LED3_OFF()    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET)
#define LED3_ON()     HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET)
#define	LED3_TOGGLE() HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin)

/* Exported functions ------------------------------------------------------- */


#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

