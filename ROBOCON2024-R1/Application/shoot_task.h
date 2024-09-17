/**
  ******************************************************************************
  * @file    shoot_task.h
  * @author  
  * @version 
  * @date   
  * @brief   This file contains the headers of shoot_task.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_can.h"
#include "struct_typedef.h"
#include "c620.h"
#include "can_odrive.h"
#include "can_damiao.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_rc.h"
#include "cmsis_os.h"
//#include "remote_task.h"
/* Exported types ------------------------------------------------------------*/

typedef enum
{
	SHOOT_MANUAL=0,
	SHOOT_AUTO=1,
	SHOOT_CMD_ERROR=2
}shoot_control_e;

typedef enum
{
	SHOOT_READY  = 0,
	
	PUT_UP       = 1,          
	PUT_DOWN     = 2,         //达秒上下
//	TRANSFER_UP  = 3,
//	TRANSFER_DOWN= 4,         //传送带上下
	SHOOT_BALL    = 3,
	
	GET_BALL_TURN = 4, 
	
	CLAMP_UP      = 5,
	CLAMP_DOWN    = 6,         //夹爪开合
	
	JAWS_SWITCH   = 7,
	
	GET_BALL_ACTION     = 8,         // 夹爪张开并放下
	SHOOT_BALL_ACTION   = 9,         //夹爪合住并翻转发射
	
	DT35_CORRECT_REDAY = 14,
	
	//蓝区
	GET_BALL1    = 15,
	SHOOT_BALL1  = 16,
	GET_BALL2    = 17,
	SHOOT_BALL2  = 18,
	GET_BALL3    = 19,
	SHOOT_BALL3  = 20,
	GET_BALL4    = 21,
	SHOOT_BALL4  = 22,
	GET_BALL5    = 23,
	SHOOT_BALL5  = 24,
	GET_BALL6    = 25,
	SHOOT_BALL6  = 26,
	GET_BALL7    = 27,
	SHOOT_BALL7  = 28,
	GET_BALL8    = 29,
	SHOOT_BALL8  = 30,
	GET_BALL9    = 31,
	SHOOT_BALL9  = 32,
	GET_BALL10   = 33,
	SHOOT_BALL10 = 34,
	GET_BALL11   = 35,
	SHOOT_BALL11 = 36,
	GET_BALL12   = 37,
	SHOOT_BALL12 = 38,
	
	//红区
	GET_BALL1_RED    = 115,
	SHOOT_BALL1_RED  = 116,
	GET_BALL2_RED    = 117,
	SHOOT_BALL2_RED  = 118,
	GET_BALL3_RED    = 119,
	SHOOT_BALL3_RED  = 120,
	GET_BALL4_RED    = 121,
	SHOOT_BALL4_RED  = 122,
	GET_BALL5_RED    = 123,
	SHOOT_BALL5_RED  = 124,
	GET_BALL6_RED    = 125,
	SHOOT_BALL6_RED  = 126,
	GET_BALL7_RED    = 127,
	SHOOT_BALL7_RED  = 128,
	GET_BALL8_RED    = 129,
	SHOOT_BALL8_RED  = 130,
	GET_BALL9_RED    = 131,
	SHOOT_BALL9_RED  = 132,
	GET_BALL10_RED   = 133,
	SHOOT_BALL10_RED = 134,
	GET_BALL11_RED   = 135,
	SHOOT_BALL11_RED = 136,
	GET_BALL12_RED   = 137,
	SHOOT_BALL12_RED = 138,

}shoot_status_e;

typedef enum
{
	SHOOT_BLUE = 0,
	SHOOT_RED  = 1,
}shoot_court_e;

typedef struct 
{
  PID_TypeDef clamp_motor_pos;
	PID_TypeDef clamp_motor_vel;
}dji_t;

typedef struct 
{
//  const rc_info_t *shoot_RC;
	const rc_info_t *shoot_RC;
	dji_t transfer[2];
	float turn_motor_pos;
	shoot_control_e control_mode;
	shoot_status_e status;
	uint8_t zone;
}shoot_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define PID_DEAD 5000.0f

void shoot_task(void const * argument);
void shoot_turn(float pos);

#endif

/****************** (C) COPYRIGHT 2024 EPOCH *****END OF FILE*************/

