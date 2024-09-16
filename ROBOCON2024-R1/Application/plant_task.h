/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLANT_TASK_H
#define __PLANT_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "servo.h"
#include "c620.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
//#include "remote_task.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "bsp_rc.h"

/* Exported types ------------------------------------------------------------*/

#define PLANT_FINISH 1

typedef enum
{
	PLANT_MANUAL=0,
	PLANT_AUTO=1,
	PLANT_CMD_ERROR=2
}plant_control_e;

typedef enum
{
	PLANT_READY=0,
	//GET_LEFT1和GET_LEFT2所作的操作相同，定义两个枚举只是为区分顺序，其他几个枚举也类似
	GET_LEFT1         = 1, 
	GET_RIGHT1        = 2,
	PUT_SIDES_RIGHT1  = 3,
	PUT_CENTER_RIGHT1 = 4,
	PUT_CENTER_LEFT1  = 5,
	PUT_SIDES_LEFT1   = 6,
	GET_RIGHT2        = 7,
	GET_LEFT2         = 8,
	PUT_SIDES_RIGHT2  = 9,
	PUT_CENTER_RIGHT2 = 10,
	PUT_CENTER_LEFT2  = 11,
	PUT_SIDES_LEFT2   = 12,
	
	GET_LEFT1_RED         = 101, 
	GET_RIGHT1_RED        = 102,
	PUT_SIDES_RIGHT1_RED  = 103,
	PUT_CENTER_RIGHT1_RED = 104,
	PUT_CENTER_LEFT1_RED  = 105,
	PUT_SIDES_LEFT1_RED   = 106,
	GET_RIGHT2_RED        = 107,
	GET_LEFT2_RED         = 108,
	PUT_SIDES_RIGHT2_RED  = 109,
	PUT_CENTER_RIGHT2_RED = 110,
	PUT_CENTER_LEFT2_RED  = 111,
	PUT_SIDES_LEFT2_RED   = 112,
	
	GET_SINGLE = 38,
	PUT_SINGLE = 39,
}plant_status_e;

typedef enum
{
	PLANT_BLUE = 0,
	PLANT_RED  = 1,
}plant_court_e;

typedef struct 
{
	const rc_info_t *plant_RC;
	//R1左右各3个舵机,贴红胶带的为舵机0,贴黄胶带为舵机1，贴黑胶带为舵机2
  servo_t servo_left[SERVO_NUM];
	servo_t servo_right[SERVO_NUM];
	plant_control_e control_mode;
	plant_status_e status;
	uint8_t zone;
	uint8_t game;
	//plant_court_e court;
}plant_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

void plant_task(void *argument);

#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

