/**
****************************(C) COPYRIGHT 2023 EPOCH****************************
* @file       ops_receive.c/h
* @brief      定位系统数据解析，发送更新位置信息指令.
* @note
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Apr-06-2023     Junshuo         1. 移植。
*
****************************(C) COPYRIGHT 2023 EPOCH****************************
*/

#include "ops_receive.h"
#include "usart.h"
#include "stdio.h"
#include "chassis_task.h"
#include "dt35.h"
#define ABS(x)        ( (x>0) ? (x) : (-x) )
#define ANGLE_PERIOD 360
extern uint8_t rx_buff_data[data_lenth];
extern chassis_move_t chassis_move;
uint8_t rec;
static void Stract (char strDestination[], char strSource[], int num);
void get_total_yaw (ops_t *p);

/**
 * @brief  数据解析函数
 * @param  rec 串口接收到的字节数据
 * @param  chassis_ops 底盘结构体
 */
void ops_data_analyse(uint8_t rec, ops_t *ops)
{
	static uint8_t ch;
	static union
	{
		uint8_t date[24];
		float ActVal[6];
	}posture;
	static uint8_t count=0;
	static uint8_t i=0;

	ch=rec;
	switch(count)
	{
		case 0:
			if(ch==0x0d)
				count++;
			else
				count=0;
			break;
		case 1:
			if(ch==0x0a)
			{
				i=0;
				count++;
			}
			else if(ch==0x0d);
			else
				count=0;
			break;
		case 2:
			posture.date[i]=ch;
			i++;
			if(i>=24)
			{
				i=0;
				count++;
			}
			break;
		case 3:
			if(ch==0x0a)
				count++;
			else
				count=0;
			break;
		case 4:
			if(ch==0x0d)
			{
				//ops的逆时针为正，与wz_set方向相反，car_yaw取负号
				//机械组把ops装反了，pos_x,pos_y取负号
				//TODO:后续考虑写一个算法，无论机械怎么装，都可以将其转化为世界坐标系
				ops->car_yaw=-posture.ActVal[0];
				ops->angle_x=posture.ActVal[1];//没有返回值
				ops->angle_y=posture.ActVal[2];//没有返回值
				ops->pos_x=-posture.ActVal[3];
				ops->pos_y=-posture.ActVal[4];
				ops->w_z=posture.ActVal[5];
				get_total_yaw(ops);
			}
			count=0;
			break;
		default:
			count=0;
		break;
	}
}
float GetX(ops_t *chassis_ops)
{
	return chassis_ops->pos_x;
}
float GetY(ops_t *chassis_ops)
{
	return chassis_ops->pos_y;
}

 
/**
 * @brief 重置定位系统
 */
void ops_reset()
{
	static char reset[4]="ACT0";
	HAL_UART_Transmit(&OPS_USART, (uint8_t *)reset, 4, HAL_MAX_DELAY); 

}


/**
 * @brief 更新X坐标
 * @param  New_X 新坐标值
 */
void Update_X(float New_X)
{
	//这个函数传入的参数为正数,更新的坐标为负数,原因未知,先直接在坐标前加一个负号
	New_X = - New_X;
	char Update_x[8]="ACTX";
	static union
	{
		float X;
		char data[4];
	}New_set;//联合体，用于将float转换为char

	New_set.X = New_X;
	Stract (Update_x,New_set.data,4);
	
  for (int i = 0; i < 8; i++) 
	{
    //while (HAL_UART_GetState(&OPS_USART) != HAL_UART_STATE_READY); 
    HAL_UART_Transmit(&OPS_USART, (uint8_t *)&Update_x[i], 1, HAL_MAX_DELAY); 
  }

}


/**
 * @brief 更新Y坐标
 * @param  New_Y 新坐标值
 */
void Update_Y(float New_Y)
{
	//这个函数传入的参数为正数,更新的坐标为负数,原因未知,先直接在坐标前加一个负号
	New_Y = - New_Y;
	char Update_y[8]="ACTY";
	static union
	{
		float Y;
		char data[4];
	}New_set;

	New_set.Y = New_Y;
	Stract (Update_y,New_set.data,4);
	
  for (int i = 0; i < 8; i++) 
	{
    //while (HAL_UART_GetState(&OPS_USART) != HAL_UART_STATE_READY); 
    HAL_UART_Transmit(&OPS_USART, (uint8_t *)&Update_y[i], 1, HAL_MAX_DELAY); 
  }

}

/**
 * @brief 更新航向角
 * @param  New_A 新坐标值
 */
void Update_A(float New_A)
{
	char update_a[8]="ACTJ";
	static union
	{
		float A;
		char data[4];
	}New_set;

	New_set.A = New_A;
	Stract (update_a,New_set.data,4);
	
	for (int i = 0; i < 8; i++) 
	{
		while (HAL_UART_GetState(&OPS_USART) != HAL_UART_STATE_READY); 
		HAL_UART_Transmit(&OPS_USART, (uint8_t *)&update_a[i], 1, HAL_MAX_DELAY); 
	}

}

/**
 * @brief 字符串拼接
 * @param  strDestination 目标字符串
 * @param  strSource 源字符串
 * @param  num 拼接长度
*/
void Stract (char strDestination[], char strSource[], int num)
{
	int i = 0;
	int j = 0;

	while (strDestination[i] != '\0')
	{
		i++;
	}

	for ( j=0; j < num; j++)
	{
		strDestination[i++]=strSource[j];
	}

}
/**
  * @brief  累加航向角
  * @param  p ops返回值结构体                  
  * @retval None
  */
void get_total_yaw (ops_t *p)
{
		float res1, res2, delta;

		if(p->car_yaw < p->car_yaw_last)
		{                        //可能的情况
			res1 = p->car_yaw + ANGLE_PERIOD - p->car_yaw_last;        	//正转，delta=+
			res2 = p->car_yaw - p->car_yaw_last;                       	//反转  delta=-
		}
		else
		{        //ecd > last
			res1 = p->car_yaw - ANGLE_PERIOD - p->car_yaw_last ;				//反转  delta -
			res2 = p->car_yaw - p->car_yaw_last;                       	//正转  delta +
		}
		//不管正反转，肯定是转的角度小的那个是真的	
		if(ABS(res1)<ABS(res2))
		{
			delta = res1;
		}
		else
		{
			delta = res2;
		}

		p->car_total_yaw += delta;//做了角度的累加，使输入角度可以不限制于360°以内
    p->car_yaw_last = p->car_yaw;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
 {

	if(USART1 == UartHandle->Instance)
	{
		dt35_data_parsing();
	}
	HAL_UART_Receive_IT(&dt35_huart, rx_buff_data, 30);

   if(UartHandle->Instance==UART5)
   {
    ops_data_analyse(rec, &chassis_move.ops);
 		HAL_UART_Receive_IT(&OPS_USART,&rec,1);
		 
   }
 }

