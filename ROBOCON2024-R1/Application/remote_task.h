/**
  ******************************************************************************
  * @file    remote_task.h
  * @author  
  * @version 
  * @date   
  * @brief   This file contains the headers of remote_task.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REMOTE_TASK_H
#define REMOTE_TASK_H
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define SW_UP     1
#define SW_MIDDLE 0
#define SW_DOWN   2

typedef enum {
	NONE_PRESS = 0, 
	SINGLE_CLICK,
	DOUBLE_CLICK,
	LONG_PRESS,
}PressEvent;

typedef union
{
	struct  {
	  int16_t ch1     : 11;  //左边的摇杆前后
		uint16_t sw1    : 2;  //右边拨码开关
		uint16_t sw2    : 2;  //左边拨码开关
									  
	  int16_t ch2     : 11;  //左边摇杆左右
		uint16_t key1   : 2;  //右边第一排第一个
		uint16_t key2   : 2;  //右边第二排第一个
									  
	  int16_t ch3     : 11;  //右边摇杆左右
		uint16_t key3   : 2;  //右边第二排第二个
		uint16_t key4   : 2;  //右边第一排第二个
									  
	  int16_t ch4     : 11;  //右边摇杆前后
		uint16_t key5   : 2;  //右上方下按键
		uint16_t key6   : 2;	 //右上方上按键
									  
	  int16_t ch5     : 11;  //右边的电位器
		uint16_t key7   : 2;  //左边第一排第一个
		uint16_t key8   : 2;	 //左边第二排第一个
									  
	  int16_t  ch6    : 11;  //待定
		uint16_t key9   : 2;  //左边第一排第二个
		uint16_t key10  : 2;  //左边第二排第二个
									  
		uint8_t key11   : 2;  //左上方上按键
		uint8_t key12   : 2;	 //左下方下按键
		uint8_t zone    : 1;	 
    uint8_t reserve : 3;
		
    uint8_t chassis_status;
		uint8_t chassis_target;
//	  int16_t ch7   : 12;
//	  int16_t reserve   : 4;  // 保留位
	}remote;
  // 打包数据
  uint8_t packed[15];
} rc_info1_t;

typedef union 
{
	struct  
	{
		uint32_t chassis_auto_status;
		uint32_t chassis_manual_status;
		uint32_t shoot_status;
		uint32_t plant_status;
		uint32_t chassis_target;
		float chassis_x;
		float chassis_y;
		float chassis_yaw;
	}robot_status; 
  uint8_t packed[32];   // 打包数据
}robot_status_t;

const rc_info1_t *get_remote_point(void);

#endif

/****************** (C) COPYRIGHT 2024 EPOCH *****END OF FILE*************/

