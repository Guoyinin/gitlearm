/**
  ******************************************************************************
  * @file     robot_task.c
  * @author   
  * @version  
  * @date     
  * @brief    用于任务之间通信
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "robot_task.h"
#include "cmsis_os.h"
#include "semphr.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
//#define CHASSIS_CORRECT_TIME 1000 //给底盘一个纠偏的时间然后上层机构再去执行相应的动作
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
/* Extern   variables ---------------------------------------------------------*/
extern osMessageQId RobotToChassisHandle;
extern osMessageQId RobotToPlantHandle;
extern osMessageQId ChassisToRobotHandle;
extern osMessageQId PlantToRobotHandle;
extern osMessageQId RobotToShootHandle;
extern osMessageQId ShootToRobotHandle;

extern osSemaphoreId RobotChassisSemphoreHandle;
extern osSemaphoreId RobotPlantSemphoreHandle;
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
uint32_t robot_test1 = 0;
uint32_t robot_test2 = 0;
/**
	* @brief  作为消息中转站，协调各个任务
  */
void robot_task(void const * argument) 
{
	uint32_t rx_data;
	uint32_t tx_data;
	while(1)
	{
//		vTaskDelay(100);
		if(xQueueReceive(ChassisToRobotHandle,&rx_data,0) == pdTRUE)
		{
			//底盘跑到点，上层机构执行这个点对应的动作
			tx_data = rx_data; 
			if(tx_data < 100) //判断红区还是蓝区
			{
				//判断底盘在一区还是二区
				if(tx_data <= 12)
				{
					xQueueSend(RobotToPlantHandle,&tx_data,0);
					robot_test1++;
				}
				else if(tx_data >=14)
				{
					xQueueSend(RobotToShootHandle,&tx_data,0);
					robot_test1=rx_data;
					robot_test2++;
				}
			}else
			{
				//判断底盘在一区还是二区
				if(tx_data <= 112)
				{
					xQueueSend(RobotToPlantHandle,&tx_data,0);
					robot_test1++;
				}
				else if(tx_data >= 114)
				{
					xQueueSend(RobotToShootHandle,&tx_data,0);
					robot_test1=rx_data;
					robot_test2++;
				}
			}
		}
		else if(xQueueReceive(PlantToRobotHandle,&rx_data,0) == pdTRUE)
		{
			//这个点的取苗动作完成后，底盘向下一个点跑,因此要自加后再赋值
			tx_data = ++rx_data;
			xQueueSend(RobotToChassisHandle,&tx_data,0);
			robot_test2++;
		}
		else if(xQueueReceive(ShootToRobotHandle,&rx_data,0) == pdTRUE)
		{
			tx_data = ++rx_data;
			xQueueSend(RobotToChassisHandle,&tx_data,0);
			robot_test2++;
		}
	}
}

/************************ (C) COPYRIGHT 2024 EPOCH *****END OF FILE****/
