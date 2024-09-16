/**
  ******************************************************************************
  * @file    can_damiao.c
  * @author  
  * @version 
  * @date    
  * @brief   This file contains the headers of can_damiao.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_DAMIAO_H
#define __CAN_DAMIAO_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "struct_typedef.h"
#include "can.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define  DAMIAO_CAN hcan2
//#define  DAMIAO_ID  01


#define  P_MIN  -12.5
#define  P_MAX  12.5
#define  V_MIN  -45
#define  V_MAX  45
#define  KP_MIN 0
#define  KP_MAX 500
#define  KD_MIN 0
#define  KD_MAX 5
#define  T_MIN -18
#define  T_MAX +18 

typedef struct
{
  fp32 speed_rpm_set;
  fp32 pos_set;

} damiao_motor_t;
/* Exported functions ------------------------------------------------------- */
void ctrl_motor( uint16_t id,float _pos, float _vel,float _KP, float _KD, float _torq);
void ctrl_motor_in(uint16_t id);
void ctrl_motor_out(uint16_t id);
void ctrl_motor_zero(uint16_t id);
void ctrl_motor_clear(uint16_t id);

#endif


/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

