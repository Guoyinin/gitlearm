/**
****************************(C) COPYRIGHT 2023 EPOCH****************************
* @file       ops_receive.c/h
* @brief      接收并处理Action平面定位系统OPS通过串口发送的数据；
* @note
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Apr-06-2023     Junshuo         1. 移植。
*
****************************(C) COPYRIGHT 2023 EPOCH****************************
*/

#ifndef OPS_RECEIVE_H
#define OPS_RECEIVE_H

#include "struct_typedef.h"

#define OPS_USART huart5

typedef struct
{
	fp32 w_z;     			//由ops获取的底盘航向角速度
  fp32 pos_x;   			//由ops获取的底盘位置x
  fp32 pos_y;   			//由ops获取的底盘位置y
  fp32 angle_x; 			//没用，ops没有返回这个值
  fp32 angle_y; 			//没用，ops没有返回这个值
	fp32 car_yaw; 			//由ops获取的底盘航向角（底盘坐标系）
	fp32 car_yaw_last;	//上一次的航向角
	fp32 car_total_yaw; //航向角累计值
}ops_t;

void ops_data_analyse(uint8_t rec, ops_t *ops);
void ops_reset(void);
float GetX(ops_t *chassis_ops);
float GetY(ops_t *chassis_ops);
void Update_X(float New_X);
void Update_Y(float New_Y);
void Update_A(float New_A);

#endif
