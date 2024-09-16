/**
  ******************************************************************************
  * @file     shoot_task.c
  * @author
  * @version
  * @date
  * @brief    发射任务，包括云台的控制、发射机构的控制
  ******************************************************************************
  * @attention
  *
  *
  *
  *
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------*/
#include "shoot_task.h"
#include "main.h"
#include "gpio.h"
#include "can_odrive.h"
#include "can_damiao.h"
#include "bsp_rc.h"
#include "pid.h"
//#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "chassis_task.h"
//#include "tim.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
#define	odrive_speed 20
#define SHOOT_TASK_INIT_TIME 10
#define SHOOT_TASK_TIME 1000   //每个动作之间的间隔时间
#define SHOOT_TASK_TIME_MS 1

//摩擦轮速度设定
//下面两个摩擦轮
#define SHOOT_VEL_SET1 26.5
#define	SHOOT_VEL_SET2 22.0
#define SHOOT_VEL_SET3 21.75
#define SHOOT_VEL_SET4 21.75
#define SHOOT_VEL_SET5 20
#define SHOOT_VEL_SET6 20
//上面一个摩擦轮
#define SHOOT_VEL2_SET1 26.5
#define	SHOOT_VEL2_SET2 22.0
#define SHOOT_VEL2_SET3 21.75
#define SHOOT_VEL2_SET4 21.75
#define SHOOT_VEL2_SET5 20
#define SHOOT_VEL2_SET6 20

//达秒电机翻转
#define	DAMIAO_TURN -2.2f
#define	DAMIAO_BACK 0.0f

//夹爪
#define CLAMP_OPEN -140000
#define CLAMP_CLOSE 0

//传送
#define TRANSFER_UP1 550000
#define TRANSFER_DOWN1 0

#define ABS(x) ((x) > 0 ? (x) : -(x))

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
/* Private  variables ---------------------------------------------------------*/
		axis_t odrive_friction[3];  // 发射摩擦轮
/* Extern   variables ---------------------------------------------------------*/
extern chassis_move_t chassis_move;

shoot_t shoot;
float odrive_vel;	
float odrive_vel2;
extern osMessageQId RobotToShootHandle;
extern osMessageQId ShootToRobotHandle;
extern rc_info_t rc;
#if INCLUDE_uxTaskGetStackHighWaterMark
  uint32_t shoot_high_water;
#endif
/* Extern   function prototypes -----------------------------------------------*/
void shoot_init(shoot_t *shoot_move_init);
void shoot_vel(float vel,float vel2);
static void shoot_set_terminal(shoot_t *shoot_terminal);
static void shoot_control_loop(shoot_t *shoot_control);
void shoot_turn(float pos);

//static void plant_set_terminal(plant_t *plant_terminal);
//static void plant_control_loop(plant_t *plant_control);

/* Private  function prototypes -----------------------------------------------*/
		
/* Private  functions ---------------------------------------------------------*/
		
/* USER CODE BEGIN 4 */
void shoot_task(void const*argument)
{ 
	shoot_init(&shoot);
	vTaskDelay(SHOOT_TASK_INIT_TIME);
	while(1)
	{
		shoot_set_terminal(&shoot);
		shoot_control_loop(&shoot);
		vTaskDelay(SHOOT_TASK_TIME_MS);
	}
}

		
 void shoot_init(shoot_t *shoot_move_init)
{
	shoot_move_init->shoot_RC = get_remote_control_point();
	
//	shoot_move_init->shoot_RC = get_remote_point();
	
	// 初始化ODrive电机 摩擦轮
	odrive_friction[0].AXIS_ID = 0x011;
	odrive_friction[1].AXIS_ID = 0x012;
	odrive_friction[2].AXIS_ID = 0x013;

	
	pid_init(&shoot.transfer[0].clamp_motor_pos);
	shoot.transfer[0].clamp_motor_pos.f_param_init(&shoot.transfer[0].clamp_motor_pos, PID_Position, 
													   10000.0f,0,5.0f,0,0,0,0.3f,0.0f,0.15f,PID_IMPROVE_NONE);
	pid_init(&shoot.transfer[0].clamp_motor_vel);
	shoot.transfer[0].clamp_motor_pos.f_param_init(&shoot.transfer[0].clamp_motor_vel, PID_Speed, 
													   10000.0f,0,5.0f,0,0,0,2.0f,0.03f,0.0f,PID_IMPROVE_NONE);
	
	pid_init(&shoot.transfer[1].clamp_motor_pos);
	shoot.transfer[1].clamp_motor_pos.f_param_init(&shoot.transfer[1].clamp_motor_pos, PID_Position, 
													   10000.0f,0,5.0f,0,0,0,0.3f,0.0f,0.15f,PID_IMPROVE_NONE);
	pid_init(&shoot.transfer[1].clamp_motor_vel);
	shoot.transfer[1].clamp_motor_pos.f_param_init(&shoot.transfer[1].clamp_motor_vel, PID_Speed, 
													   10000.0f,0,5.0f,0,0,0,2.0f,0.03f,0.0f,PID_IMPROVE_NONE);

	shoot_vel(20,10);
	vTaskDelay(2000);
	shoot_vel(20,10);
	
  // 初始化达妙电机
	vTaskDelay(3000);
	ctrl_motor_in(0x01);
	ctrl_motor_zero(0x01);

//	shoot.turn_motor_pos = DAMIAO_BACK;
	shoot_turn(DAMIAO_BACK);
	
	vTaskDelay(16000);
	shoot.transfer[0].clamp_motor_pos.target = CLAMP_OPEN;

}

/**
  * @brief  5065摩擦轮速度设置
  */
//float vel = 20;
void shoot_vel(float vel,float vel2)
{
	odrv_Set_Input_Vel(odrive_friction[0],-vel, 0);
	odrv_Set_Input_Vel(odrive_friction[1],-vel, 0);
	odrv_Set_Input_Vel(odrive_friction[2],vel2, 0);
}

///**
//  * @brief  4250
//  */
////float pos=0.3;
//void shoot_pos(float pos)
//{
////	odrv_Set_Input_Pos(odrive_transfer[0],0, 0, 0);
////	vTaskDelay(1000);
//	odrv_Set_Input_Pos(odrive_friction[2],pos, 0, 0);
//}

/**
  * @brief  4310夹爪翻转
  */
void shoot_turn(float pos)
{
	ctrl_motor(0x01,pos, 0, 6, 0.8, 0);
}

/**
  * @brief  2006 target=140000开 targt=0关
  */
void clamp_move(float target)
{
	shoot.transfer[0].clamp_motor_pos.target = target;
}

/**
  * @brief          处理手摇状态下的按键指令
  * @param[out]     shoot_terminal 变量指针.
  * @retval         none
  */
static void shoot_key_handle(shoot_t *shoot_key)
{
//	if(shoot_key->zone == SHOOT_BLUE)
//	{
static uint8_t i = 0;
i++;
if(10 == i)
{
		
	switch(shoot_key->shoot_RC->mKey)
	{
		case KEY_R_L:
			shoot_key->status = SHOOT_BALL;
			break;
		case KEY_L_U:
			shoot_key->status = GET_BALL_TURN;
			break;
		case KEY_L_D:
			shoot_key->status = JAWS_SWITCH;
			break;
		case KEY_R_U:
			shoot_key->status = SHOOT_BALL_ACTION;
		break;
		case KEY_R_D:
			shoot_key->status = GET_BALL_ACTION;
		break;
		}
	i = 0;

//		case KEY_L_L:
//			chassis_move.zero_set = 1;
//			break;
//		case KEY_R_R:
//			shoot_key->status = CLAMP_UP;
//			break;
//		case KEY_R_L:
//			shoot_key->status = CLAMP_DOWN;
//			break;

//	}
		
		
//		if(shoot_key->shoot_RC->remote.key7 == SINGLE_CLICK)
//		{
//			shoot_key->status = PUT_UP;
//		}
//		else if(shoot_key->shoot_RC->remote.key8 == SINGLE_CLICK)
//		{
//			shoot_key->status = PUT_DOWN;
//		}
//		else if(shoot_key->shoot_RC->remote.key10 == SINGLE_CLICK)
//		{
//			shoot_key->status = TRANSFER_UP;
//		}
//		else if(shoot_key->shoot_RC->remote.key1 == SINGLE_CLICK)
//		{
//			shoot_key->status = TRANSFER_DOWN;
//		}
//		else if(shoot_key->shoot_RC->remote.key2 == SINGLE_CLICK)
//		{
//			shoot_key->status = CLAMP_UP;
//		}
//		else if(shoot_key->shoot_RC->remote.key3 == SINGLE_CLICK)
//		{
//			shoot_key->status = CLAMP_DOWN;
//		}
	}

	
/**************************************************************/	
//	if(shoot_key->shoot_RC->remote.zone == SHOOT_BLUE)
//	{
//		if(shoot_key->shoot_RC->remote.key7 == SINGLE_CLICK)
//		{
//			shoot_key->status = PUT_UP;
//		}
//		else if(shoot_key->shoot_RC->remote.key8 == SINGLE_CLICK)
//		{
//			shoot_key->status = PUT_DOWN;
//		}
//		else if(shoot_key->shoot_RC->remote.key10 == SINGLE_CLICK)
//		{
//			shoot_key->status = TRANSFER_UP;
//		}
//		else if(shoot_key->shoot_RC->remote.key1 == SINGLE_CLICK)
//		{
//			shoot_key->status = TRANSFER_DOWN;
//		}
//		else if(shoot_key->shoot_RC->remote.key2 == SINGLE_CLICK)
//		{
//			shoot_key->status = CLAMP_UP;
//		}
//		else if(shoot_key->shoot_RC->remote.key3 == SINGLE_CLICK)
//		{
//			shoot_key->status = CLAMP_DOWN;
//		}
//	}
//	else if(shoot_key->shoot_RC->remote.zone == SHOOT_RED)
//	{
//		if(shoot_key->shoot_RC->remote.key7 == SINGLE_CLICK)
//		{
//			shoot_key->status = PUT_UP;
//		}
//		else if(shoot_key->shoot_RC->remote.key8 == SINGLE_CLICK)
//		{
//			shoot_key->status = PUT_DOWN;
//		}
//		else if(shoot_key->shoot_RC->remote.key10 == SINGLE_CLICK)
//		{
//			shoot_key->status = TRANSFER_UP;
//		}
//		else if(shoot_key->shoot_RC->remote.key1 == SINGLE_CLICK)
//		{
//			shoot_key->status = TRANSFER_DOWN;
//		}
//		else if(shoot_key->shoot_RC->remote.key2 == SINGLE_CLICK)
//		{
//			shoot_key->status = CLAMP_UP;
//		}
//		else if(shoot_key->shoot_RC->remote.key3 == SINGLE_CLICK)
//		{
//			shoot_key->status = CLAMP_DOWN;
//		}
//	}
}

/**
  * @brief          取球动作控制台，设置控制状态
  * @param[out]     shoot_terminal 变量指针.
  * @retval         none
  */
static void shoot_set_terminal(shoot_t *shoot_terminal)
{
	if(shoot_terminal->shoot_RC->sw[RC_SW_L_CHANNEL] == RC_SW_UP && shoot_terminal->shoot_RC->sw[RC_SW_R_CHANNEL] == RC_SW_UP)
	{
		shoot_terminal->control_mode = SHOOT_MANUAL; 
		shoot_key_handle(shoot_terminal);
	}
	else if(shoot_terminal->shoot_RC->sw[RC_SW_L_CHANNEL] == RC_SW_UP )
	{
		shoot_terminal->control_mode = SHOOT_AUTO;
		uint32_t rx_buff;
		if(xQueueReceive(RobotToShootHandle,&rx_buff,0) == pdTRUE)
		{
			shoot_terminal->status = rx_buff;
		}
	}
	else
	{
		shoot_terminal->control_mode = SHOOT_CMD_ERROR;
	}

/***************************************************************************************************/	
////	static uint32_t rx_buff;
//	if(shoot_terminal->shoot_RC->remote.sw2 == SW_UP && shoot_terminal->shoot_RC->remote.sw1 == SW_UP)
//	{
//		shoot_terminal->control_mode = SHOOT_MANUAL; 
//		shoot_key_handle(shoot_terminal);
//	}
//	else if(shoot_terminal->shoot_RC->remote.sw2 == SW_UP )
//	{
//		shoot_terminal->control_mode = SHOOT_AUTO;
//		uint32_t rx_buff;
//		if(xQueueReceive(RobotToShootHandle,&rx_buff,0) == pdTRUE)
//		{
//			shoot_terminal->status = (shoot_status_e)rx_buff;
//		}
//	}
//	else
//	{
//		shoot_terminal->control_mode = SHOOT_CMD_ERROR;
//	}
}

/**
  * @brief          执行取球动作
  * @param[out]     plant_control 变量指针.
  * @retval         none
  */
static void shoot_control_loop(shoot_t *shoot_control)
{
	uint32_t tx_data = shoot_control->status;
	 odrive_vel = 20+rc.ch[5]*0.025;
	 odrive_vel2 = 10+rc.ch[6]*0.025;
	switch(shoot_control->status)
	{
		case SHOOT_READY:
			break;
		case PUT_UP:
			shoot_control->turn_motor_pos = DAMIAO_TURN;
//			shoot_turn(DAMIAO_TURN);
			vTaskDelay(1);		
			break;
		case PUT_DOWN:
			shoot_control->turn_motor_pos = DAMIAO_BACK;
//			shoot_turn(DAMIAO_BACK);
			vTaskDelay(1);
			break;
		case SHOOT_BALL:
//			shoot_vel(odrive_vel,odrive_vel2);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_UP1;
			vTaskDelay(1000);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_DOWN1;	
			break;
		case CLAMP_UP:
			shoot.transfer[0].clamp_motor_pos.target = CLAMP_CLOSE;
			vTaskDelay(1);
			break;
		case CLAMP_DOWN:
			shoot.transfer[0].clamp_motor_pos.target = CLAMP_OPEN;
			vTaskDelay(1);
			break;
		case JAWS_SWITCH:
			if(shoot.transfer[0].clamp_motor_pos.target == CLAMP_OPEN)
			{
				shoot.transfer[0].clamp_motor_pos.target = CLAMP_CLOSE;
			}else if(shoot.transfer[0].clamp_motor_pos.target == CLAMP_CLOSE)
			{
				shoot.transfer[0].clamp_motor_pos.target = CLAMP_OPEN;
			}
			vTaskDelay(100);
			break;
		case GET_BALL_TURN:
			if(shoot_control->turn_motor_pos == DAMIAO_BACK)
			{
				shoot_control->turn_motor_pos = DAMIAO_TURN;
//				shoot_turn(DAMIAO_TURN);

				break;
			  vTaskDelay(100);
			}else if(shoot_control->turn_motor_pos < 0.0f)
			{
				shoot_control->turn_motor_pos = DAMIAO_BACK;
//				shoot_turn(DAMIAO_BACK);
			}
			break;
		case GET_BALL_ACTION:
//		  if(ABS(shoot.transfer[0].clamp_motor_pos.measure - CLAMP_OPEN) < PID_DEAD)
//			{
//				shoot.transfer[0].clamp_motor_pos.target = CLAMP_OPEN;
//			  vTaskDelay(600);
				shoot_control->turn_motor_pos = DAMIAO_TURN;
//			  shoot.transfer[0].clamp_motor_pos.target = CLAMP_CLOSE;
//			}else{
//						shoot.transfer[0].clamp_motor_pos.target = CLAMP_OPEN;
//			}
				shoot_turn(shoot.turn_motor_pos);
			break;
			
		case SHOOT_BALL_ACTION:
			
			shoot.transfer[0].clamp_motor_pos.target = CLAMP_CLOSE;
			vTaskDelay(400);
		
			shoot_control->turn_motor_pos = DAMIAO_BACK;
				shoot_turn(shoot.turn_motor_pos);
			vTaskDelay(500);
			shoot.transfer[0].clamp_motor_pos.target = CLAMP_OPEN;
//		  if(ABS(shoot.transfer[0].clamp_motor_pos.measure - CLAMP_OPEN) < PID_DEAD)
//			{
//			vTaskDelay(700);
//				shoot.transfer[1].clamp_motor_pos.target = TRANSFER_UP1;
////				if(ABS(shoot.transfer[1].clamp_motor_pos.measure - CLAMP_OPEN) < PID_DEAD)
////				{
//			vTaskDelay(1000);
//					shoot.transfer[1].clamp_motor_pos.target = TRANSFER_DOWN1;
//				}
//			}
//			shoot.transfer[0].clamp_motor_pos.target = CLAMP_CLOSE;
			break;
			
			
		case DT35_CORRECT_REDAY:
			//shoot_control->turn_motor_pos = DAMIAO_TURN;
			shoot.transfer[0].clamp_motor_pos.target = CLAMP_OPEN;
			break;
		case GET_BALL1 :
		case GET_BALL2 :
		case GET_BALL3 :
		case GET_BALL4 :
		case GET_BALL5 :
		case GET_BALL6 :
		case GET_BALL7 :
		case GET_BALL8 :
		case GET_BALL9 :
		case GET_BALL10:
		case GET_BALL11:
		case GET_BALL12:
		case GET_BALL1_RED :
		case GET_BALL2_RED :
		case GET_BALL3_RED :
		case GET_BALL4_RED :
		case GET_BALL5_RED :
		case GET_BALL6_RED :
		case GET_BALL7_RED :
		case GET_BALL8_RED :
		case GET_BALL9_RED :
		case GET_BALL10_RED:
		case GET_BALL11_RED:
		case GET_BALL12_RED:
			shoot.transfer[0].clamp_motor_pos.target = CLAMP_CLOSE;
			vTaskDelay(SHOOT_TASK_TIME);
			shoot_control->turn_motor_pos = DAMIAO_BACK;
			vTaskDelay(SHOOT_TASK_TIME);
		  shoot.transfer[0].clamp_motor_pos.target = CLAMP_OPEN;
			vTaskDelay(SHOOT_TASK_TIME);
			shoot_control->turn_motor_pos = DAMIAO_TURN;
			xQueueSend(ShootToRobotHandle,&tx_data,0);
			break;
		case SHOOT_BALL1:
		case SHOOT_BALL7:
		case SHOOT_BALL1_RED:
		case SHOOT_BALL7_RED:
			shoot_vel(SHOOT_VEL_SET1,SHOOT_VEL2_SET1);
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_UP1;
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_DOWN1;
			break;
		case SHOOT_BALL2:
		case SHOOT_BALL8:
		case SHOOT_BALL2_RED:
		case SHOOT_BALL8_RED:
			shoot_vel(SHOOT_VEL_SET2,SHOOT_VEL2_SET2);
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_UP1;
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_DOWN1;
			break;
		case SHOOT_BALL3:
		case SHOOT_BALL9:
		case SHOOT_BALL3_RED:
		case SHOOT_BALL9_RED:
			shoot_vel(SHOOT_VEL_SET3,SHOOT_VEL2_SET3);
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_UP1;
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_DOWN1;
			break;
		case SHOOT_BALL4 :
		case SHOOT_BALL10:
		case SHOOT_BALL4_RED :
		case SHOOT_BALL10_RED:
			shoot_vel(SHOOT_VEL_SET4,SHOOT_VEL2_SET4);
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_UP1;
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_DOWN1;
			break;
		case SHOOT_BALL5:
		case SHOOT_BALL11:
		case SHOOT_BALL5_RED :
		case SHOOT_BALL11_RED:
			shoot_vel(SHOOT_VEL_SET5,SHOOT_VEL2_SET5);
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_UP1;
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_DOWN1;
			break;
		case SHOOT_BALL6:
		case SHOOT_BALL12:
		case SHOOT_BALL6_RED :
		case SHOOT_BALL12_RED:
			shoot_vel(SHOOT_VEL_SET6,SHOOT_VEL2_SET6);
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_UP1;
			vTaskDelay(SHOOT_TASK_TIME);
			shoot.transfer[1].clamp_motor_pos.target = TRANSFER_DOWN1;
			break;
		default:
			break;
	}
	shoot_control->status = SHOOT_READY;	
	vTaskDelay(2);

}


/* USER CODE END 4 */

/************************ (C) COPYRIGHT 2024 EPOCH *****END OF FILE****/
