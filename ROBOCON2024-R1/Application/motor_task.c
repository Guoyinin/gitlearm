/**
  ******************************************************************************
  * @file     led_task.c
  * @author   Junshuo
  * @version  V1.0
  * @date     Jun-28-2023
  * @brief    led????????????????????
  ******************************************************************************
  * @attention
  * ???????
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "motor_task.h"
#include "cmsis_os.h"
#include "plant_task.h"
#include "shoot_task.h"
//#include "chassis_task.h"
//#include "tim.h"
#include "c620.h"
#include "chassis_task.h"

/* Private  typedef -----------------------------------------------------------*/
int motor_task_run_count = 0;
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
#define MOTOR_TASK_INIT_TIME 500
#define MOTOR_CONTROL_TIME_MS 2
/* Private  variables ---------------------------------------------------------*/
void chassis_run(chassis_move_t *chassis_run);
/* Extern   variables ---------------------------------------------------------*/
//extern dji_motor_t plant_motor
extern chassis_move_t chassis_move;
extern plant_t plant[2];
extern shoot_t shoot;
extern motor_measure_t motor_chassis[4];
extern motor_measure_t motor_plant[2];
extern motor_measure_t motor_shoot[2];

/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/


/**
 * @brief 底盘3508电机控制
 * @param chassis_run 底盘结构体
 */
void chassis_run(chassis_move_t *chassis_run) 
{ 
	for(uint8_t i=0;i<4;i++)
	{
		chassis_run->motor_pid[i].target = chassis_run->wheel_velocity[i];
		chassis_run->motor_pid[i].f_cal_pid(&chassis_run->motor_pid[i], motor_chassis[i].speed_rpm);
	}
  CAN_cmd_chassis((int16_t)chassis_run->motor_pid[0].output, (int16_t)chassis_run->motor_pid[1].output,
										(int16_t)chassis_run->motor_pid[2].output,(int16_t)chassis_run->motor_pid[3].output);
//	printf("hello world\n");
//	CAN_cmd_chassis(0,0,1500,0);
}

/**
 * @brief 发送电机控制指令
 * @param 
 */
void shoot_cmd(void)
{
	for(uint8_t i=0;i<2;i++)
	{
		
		shoot.transfer[i].clamp_motor_pos.f_cal_pid(&shoot.transfer[i].clamp_motor_pos,motor_shoot[i].total_angle);
		
		shoot.transfer[i].clamp_motor_vel.target = shoot.transfer[i].clamp_motor_pos.f_cal_pid(&shoot.transfer[i].clamp_motor_pos,motor_shoot[i].total_angle);
		shoot.transfer[i].clamp_motor_vel.f_cal_pid(&shoot.transfer[i].clamp_motor_vel,motor_shoot[i].speed_rpm);
	}
	CAN_cmd_shoot((int16_t)shoot.transfer[0].clamp_motor_vel.output,(int16_t)shoot.transfer[1].clamp_motor_vel.output,0,0);
}


void motor_task(void const *pvParameters)
{
  vTaskDelay(MOTOR_TASK_INIT_TIME);
	while(1)
	{ 
		shoot_cmd();
		chassis_run(&chassis_move);
		shoot_turn(shoot.turn_motor_pos);
		//plant_cmd(plant);
//    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);  W  
		vTaskDelay(MOTOR_CONTROL_TIME_MS);
	}

}


/************************ (C) COPYRIGHT 2024 EPOCH *****END OF FILE****/

