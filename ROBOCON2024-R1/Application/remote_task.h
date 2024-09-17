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
	  int16_t ch1     : 11;  //��ߵ�ҡ��ǰ��
		uint16_t sw1    : 2;  //�ұ߲��뿪��
		uint16_t sw2    : 2;  //��߲��뿪��
									  
	  int16_t ch2     : 11;  //���ҡ������
		uint16_t key1   : 2;  //�ұߵ�һ�ŵ�һ��
		uint16_t key2   : 2;  //�ұߵڶ��ŵ�һ��
									  
	  int16_t ch3     : 11;  //�ұ�ҡ������
		uint16_t key3   : 2;  //�ұߵڶ��ŵڶ���
		uint16_t key4   : 2;  //�ұߵ�һ�ŵڶ���
									  
	  int16_t ch4     : 11;  //�ұ�ҡ��ǰ��
		uint16_t key5   : 2;  //���Ϸ��°���
		uint16_t key6   : 2;	 //���Ϸ��ϰ���
									  
	  int16_t ch5     : 11;  //�ұߵĵ�λ��
		uint16_t key7   : 2;  //��ߵ�һ�ŵ�һ��
		uint16_t key8   : 2;	 //��ߵڶ��ŵ�һ��
									  
	  int16_t  ch6    : 11;  //����
		uint16_t key9   : 2;  //��ߵ�һ�ŵڶ���
		uint16_t key10  : 2;  //��ߵڶ��ŵڶ���
									  
		uint8_t key11   : 2;  //���Ϸ��ϰ���
		uint8_t key12   : 2;	 //���·��°���
		uint8_t zone    : 1;	 
    uint8_t reserve : 3;
		
    uint8_t chassis_status;
		uint8_t chassis_target;
//	  int16_t ch7   : 12;
//	  int16_t reserve   : 4;  // ����λ
	}remote;
  // �������
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
  uint8_t packed[32];   // �������
}robot_status_t;

const rc_info1_t *get_remote_point(void);

#endif

/****************** (C) COPYRIGHT 2024 EPOCH *****END OF FILE*************/

