/**
  ******************************************************************************
  * @file     bsp_rc.h
  * @author   Junshuo
  * @version  V1.0
  * @date     Jan-26-2022
  * @brief    This file contains the headers of bsp_rc.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_RC_H__
#define __BSP_RC_H__

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  remote control information
  */
typedef __packed struct
{
	int16_t ch[7];  // 摇杆
  char sw[2];     // 钮子开关
  // char key[9];
  uint16_t mKey;    // 按键

} rc_info_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
 #define DBUS_HUART       huart3 /* for remote controler reciever */

// 钮子开关相关定义
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)0)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
#define RC_SW_R_CHANNEL 0
#define RC_SW_L_CHANNEL 1

// 按键相关定义
#define KEY_PRES 0//遥控器按下
#define KEY_UP 1//遥控器未按下
#define KEY_ROCKER ((uint16_t)1 << 8)
#define KEY_L_L ((uint16_t)1 << 1)
#define KEY_L_R ((uint16_t)1 << 2)
#define KEY_L_U ((uint16_t)1 << 3)
#define KEY_L_D ((uint16_t)1 << 0)
#define KEY_R_L ((uint16_t)1 << 6)
#define KEY_R_R ((uint16_t)1 << 4)
#define KEY_R_U ((uint16_t)1 << 7)
#define KEY_R_D ((uint16_t)1 << 5)

// 摇杆通道定义
#define RC_ROCKER_X_CHANNEL 4
#define RC_ROCKER_Y_CHANNEL 3

/* Exported functions ------------------------------------------------------- */
void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
const rc_info_t *get_remote_control_point(void);


#endif


/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

