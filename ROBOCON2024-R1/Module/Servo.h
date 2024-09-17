/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief   This file contains the headers of Servo.c
  ******************************************************************************
  * @attention
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERVO_h
#define __SERVO_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "usart.h"
//#include "tim.h"
#include <stdarg.h>
/* Exported types ------------------------------------------------------------*/



typedef enum //舵机转动的方向
{
  FORWARD,
  REVERSE
}servo_direction;


typedef struct //舵机结构体
{
 uint8_t id; //舵机的id
  uint16_t position;//舵机位置
  uint16_t angle;//舵机角度
 uint16_t time; //旋转的时间(单位ms)
  servo_direction direction;//舵机转动的方向
  
//  TIM_HandlebvTypeDef htim;
//  uint8_t tim_channel;
}servo_t;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define SERVO_NUM 3 //舵机数量
#define SERVO_TIME 100 //舵机转动的时间（单位ms) 时间选择不对可能导致三个舵机之间有延迟
/* Exported functions ------------------------------------------------------- */

extern void servo_init_lsc(servo_t *servo,uint8_t id,servo_direction direction,uint16_t time);
extern void move_servo_lsc(servo_t *servo,uint16_t angle);
// extern void move_servos(uint8_t Num, uint16_t Time, ...);
extern uint16_t calculate_position_lsc(servo_t *servo);
//extern void servo_init(servo_t *servo,TIM_HandleTypeDef htim,uint8_t tim_channel,servo_direction direction);
//extern  void move_servo(servo_t *servo,uint16_t angle);
//extern uint16_t calculate_position(servo_t *servo);

#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

