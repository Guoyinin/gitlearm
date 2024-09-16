/**
  ******************************************************************************
  * @file     plant_task.c
  * @author   Guoyinin
  * @version  v1.0
  * @date     
  * @brief    R1ȡ������
	* @note 		���� ��ȡ������ұ� ���Ҫ�ȷ������ٷ��м䣬�ұ����ȷ��м��ٷ�����
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "plant_task.h"
#include "config.h"

/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
#define PLANT_TASK_INIT_TIME 16000
#define PLANT_TASK_TIME_MS 1
#define GET_TIME 500//���̵ȴ�ȡ���õ�ʱ��
#define PUT_TIME 500//���̵ȴ������õ�ʱ��
#define GAP_TIME 500 //ȡ�����ʱ���������̧�������ļ��

//����������ȡ�����λ��
#define SERVO0_LEFT_OPEN 2500   //˫��ɫ��ƿλ�� ��
#define SERVO0_LEFT_CLOSE 750 //ֵԽС Խ��
#define SERVO1_LEFT_OPEN 2000//��ƿλ�� ��
#define SERVO1_LEFT_CLOSE 800
#define SERVO2_LEFT_OPEN 1200//��ƿλ�� ��
#define SERVO2_LEFT_CLOSE 2190//ֵԽ�� Խ��


//�ұ��������ȡ�����λ��
#define SERVO0_RIGHT_OPEN 560 //��ɫ ֵԽ�� ������
#define SERVO0_RIGHT_CLOSE 2350         //ֵԽ�� Խ��
#define SERVO1_RIGHT_OPEN 2400//��ɫ
#define SERVO1_RIGHT_CLOSE 1200 //ֵԽ�� Խ��
#define SERVO2_RIGHT_OPEN 1800//��ɫ 
#define SERVO2_RIGHT_CLOSE 540 //ֵԽ�� Խ��


//�̵�������̧��

#define ARM_RIGHT_DOWN        HAL_GPIO_WritePin(ARM_RIGHT_RISE_GPIO_Port,ARM_RIGHT_RISE_Pin,GPIO_PIN_SET)
#define ARM_RIGHT_UP          HAL_GPIO_WritePin(ARM_RIGHT_RISE_GPIO_Port,ARM_RIGHT_RISE_Pin,GPIO_PIN_RESET)
#define ARM_LEFT_DOWN         HAL_GPIO_WritePin(ARM_LEFT_RISE_GPIO_Port,ARM_LEFT_RISE_Pin,GPIO_PIN_SET)
#define ARM_LEFT_UP           HAL_GPIO_WritePin(ARM_LEFT_RISE_GPIO_Port,ARM_LEFT_RISE_Pin,GPIO_PIN_RESET)
#define ARM_LEFT_STRETCH      HAL_GPIO_WritePin(ARM_LEFT_STRETCH_GPIO_Port,ARM_LEFT_STRETCH_Pin,GPIO_PIN_SET)
#define ARM_RIGHT_STRETCH     HAL_GPIO_WritePin(ARM_RIGHT_STRETCH_GPIO_Port,ARM_RIGHT_STRETCH_Pin,GPIO_PIN_SET)
#define ARM_LEFT_SHRINK      HAL_GPIO_WritePin(ARM_LEFT_STRETCH_GPIO_Port,ARM_LEFT_STRETCH_Pin,GPIO_PIN_RESET)
#define ARM_RIGHT_SHRINK     HAL_GPIO_WritePin(ARM_RIGHT_STRETCH_GPIO_Port,ARM_RIGHT_STRETCH_Pin,GPIO_PIN_RESET)



////����������ȡ�����λ��
//#define SERVO0_LEFT_OPEN  1500
//#define SERVO0_LEFT_CLOSE 2000
//#define SERVO1_LEFT_OPEN  1500
//#define SERVO1_LEFT_CLOSE 950 
//#define SERVO2_LEFT_OPEN  2000
//#define SERVO2_LEFT_CLOSE 1400
////�ұ��������ȡ�����λ��
//#define SERVO0_RIGHT_OPEN  1500
//#define SERVO0_RIGHT_CLOSE 2300
//#define SERVO1_RIGHT_OPEN  1500
//#define SERVO1_RIGHT_CLOSE 1000
//#define SERVO2_RIGHT_OPEN  1500
//#define SERVO2_RIGHT_CLOSE 900

////�̵�������̧��������
//#define ARM_LEFT_DOWN         HAL_GPIO_WritePin(ARM_LEFT_RISE_GPIO_Port,ARM_LEFT_RISE_Pin,GPIO_PIN_RESET)
//#define ARM_LEFT_UP           HAL_GPIO_WritePin(ARM_LEFT_RISE_GPIO_Port,ARM_LEFT_RISE_Pin,GPIO_PIN_SET)
//#define ARM_RIGHT_DOWN        HAL_GPIO_WritePin(ARM_RIGHT_RISE_GPIO_Port,ARM_RIGHT_RISE_Pin,GPIO_PIN_RESET)
//#define ARM_RIGHT_UP          HAL_GPIO_WritePin(ARM_RIGHT_RISE_GPIO_Port,ARM_RIGHT_RISE_Pin,GPIO_PIN_SET)
//#define ARM_LEFT_STRETCH      HAL_GPIO_WritePin(ARM_LEFT_STRETCH_GPIO_Port,ARM_LEFT_STRETCH_Pin,GPIO_PIN_SET)
//#define ARM_RIGHT_STRETCH     HAL_GPIO_WritePin(ARM_RIGHT_STRETCH_GPIO_Port,ARM_RIGHT_STRETCH_Pin,GPIO_PIN_SET)
//#define ARM_LEFT_SHRINK      HAL_GPIO_WritePin(ARM_LEFT_STRETCH_GPIO_Port,ARM_LEFT_STRETCH_Pin,GPIO_PIN_RESET)
//#define ARM_RIGHT_SHRINK     HAL_GPIO_WritePin(ARM_RIGHT_STRETCH_GPIO_Port,ARM_RIGHT_STRETCH_Pin,GPIO_PIN_RESET)
/* Private  variables ---------------------------------------------------------*/
plant_t plant;//ȡ������ṹ��

uint32_t plant_test=0;
uint16_t servo0_test=1500;
uint16_t servo1_test=1500;
uint16_t servo2_test=1500;

/* Extern   variables ---------------------------------------------------------*/
extern osMessageQId RobotToPlantHandle;
extern osMessageQId PlantToRobotHandle;
extern osSemaphoreId RobotPlantSemphoreHandle;
/* Extern   function prototypes -----------------------------------------------*/
static void plant_init(plant_t *plant_move_init);
static void plant_set_terminal(plant_t *plant_terminal);
static void plant_control_loop(plant_t *plant_control);
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

void plant_task(void *argument)
{ 
	plant_init(&plant);
	vTaskDelay(PLANT_TASK_INIT_TIME);	
//	ARM_LEFT_DOWN;
//	ARM_RIGHT_DOWN;
//	ARM_LEFT_STRETCH;
//	ARM_RIGHT_STRETCH;

	while(1)
	{
		plant_set_terminal(&plant);
		plant_control_loop(&plant);	
		vTaskDelay(PLANT_TASK_TIME_MS);
	}
}


/**
  * @brief  ȡ�������ʼ��
  */
static void plant_init(plant_t *plant_move_init)
{
	
	plant_move_init->plant_RC = get_remote_control_point();
//	plant_move_init->plant_RC = get_remote_point();
	
	servo_init_lsc(&plant_move_init->servo_left[0],0,REVERSE,SERVO_TIME);
  servo_init_lsc(&plant_move_init->servo_left[1],1,FORWARD,SERVO_TIME);
  servo_init_lsc(&plant_move_init->servo_left[2],2,FORWARD,SERVO_TIME);
	
	servo_init_lsc(&plant_move_init->servo_right[0],13,REVERSE,SERVO_TIME);
  servo_init_lsc(&plant_move_init->servo_right[1],14,FORWARD,SERVO_TIME);
  servo_init_lsc(&plant_move_init->servo_right[2],15,FORWARD,SERVO_TIME);
	
	plant_move_init->control_mode = PLANT_MANUAL;
	plant_move_init->status = PLANT_READY;
	
	vTaskDelay(500);
	
	move_servo_lsc(&plant_move_init->servo_left[0],SERVO0_LEFT_OPEN);
	move_servo_lsc(&plant_move_init->servo_left[1],SERVO1_LEFT_OPEN);
	move_servo_lsc(&plant_move_init->servo_left[2],SERVO2_LEFT_OPEN);
	move_servo_lsc(&plant_move_init->servo_right[0],SERVO0_RIGHT_OPEN);
	move_servo_lsc(&plant_move_init->servo_right[1],SERVO1_RIGHT_OPEN);
	move_servo_lsc(&plant_move_init->servo_right[2],SERVO2_RIGHT_OPEN);
	
	plant_move_init->zone   = ZONE_VALUE;
	plant_move_init->game   = GAME_KIND;
//	plant_move_init->zone = PLANT_BLUE;	
}

/**
  * @brief  ȡ��
  */
void get_left(plant_t *plant_control)
{
	ARM_LEFT_DOWN;
	vTaskDelay(GAP_TIME);
	move_servo_lsc(&plant_control->servo_left[0],SERVO0_LEFT_CLOSE);
	move_servo_lsc(&plant_control->servo_left[1],SERVO1_LEFT_CLOSE);
	move_servo_lsc(&plant_control->servo_left[2],SERVO2_LEFT_CLOSE);
	
	vTaskDelay(GAP_TIME);
	ARM_LEFT_UP;
}

void get_right(plant_t *plant_control)
{
	ARM_RIGHT_DOWN;
	vTaskDelay(GAP_TIME);
	move_servo_lsc(&plant_control->servo_right[0],SERVO0_RIGHT_CLOSE);
	move_servo_lsc(&plant_control->servo_right[1],SERVO1_RIGHT_CLOSE);
	move_servo_lsc(&plant_control->servo_right[2],SERVO2_RIGHT_CLOSE);
	
	vTaskDelay(GAP_TIME);
	
	ARM_RIGHT_UP;
}


/**
  * @brief  �����ߵ���
  */
void put_sides_left(plant_t *plant_control)
{
	ARM_LEFT_DOWN;
	
	vTaskDelay(GAP_TIME);
	
	move_servo_lsc(&plant_control->servo_left[0],SERVO0_LEFT_OPEN);
	move_servo_lsc(&plant_control->servo_left[2],SERVO2_LEFT_OPEN);
	
	vTaskDelay(GAP_TIME);
	
	ARM_LEFT_UP;
}

void put_sides_right(plant_t *plant_control)
{
	ARM_RIGHT_DOWN;
	
	vTaskDelay(GAP_TIME);
	
	move_servo_lsc(&plant_control->servo_right[0],SERVO0_RIGHT_OPEN);
	move_servo_lsc(&plant_control->servo_right[2],SERVO2_RIGHT_OPEN);
	
	vTaskDelay(GAP_TIME);
	ARM_RIGHT_UP;
}

void put_sides_left_red(plant_t *plant_control)
{
	ARM_LEFT_DOWN;
	
	vTaskDelay(GAP_TIME);
	
	move_servo_lsc(&plant_control->servo_left[0],SERVO0_LEFT_OPEN);
	move_servo_lsc(&plant_control->servo_left[2],SERVO2_LEFT_OPEN);
	
	vTaskDelay(GAP_TIME);
	ARM_LEFT_UP;
}

void put_sides_right_red(plant_t *plant_control)
{
	ARM_RIGHT_DOWN;
	
	vTaskDelay(GAP_TIME);
	
	move_servo_lsc(&plant_control->servo_right[0],SERVO0_RIGHT_OPEN);
	move_servo_lsc(&plant_control->servo_right[2],SERVO2_RIGHT_OPEN);
	
	vTaskDelay(GAP_TIME);
	
	ARM_RIGHT_UP;	
}
/**
  * @brief  ���м����
  */
void put_center_left(plant_t *plant_control)
{
	ARM_LEFT_DOWN;
	
	vTaskDelay(GAP_TIME);
	
	move_servo_lsc(&plant_control->servo_left[1],SERVO1_LEFT_OPEN);
	
	
	vTaskDelay(GAP_TIME);
	ARM_LEFT_UP;
}

void put_center_right(plant_t *plant_control)
{
	ARM_RIGHT_DOWN;
	
	vTaskDelay(GAP_TIME);
	
	move_servo_lsc(&plant_control->servo_right[1],SERVO1_RIGHT_OPEN);
	
	vTaskDelay(GAP_TIME);
	
	ARM_RIGHT_UP;
}

void put_center_left_red(plant_t *plant_control)
{
	ARM_LEFT_DOWN;
	
	vTaskDelay(GAP_TIME);
	
	move_servo_lsc(&plant_control->servo_left[1],SERVO1_LEFT_OPEN);
	
	vTaskDelay(GAP_TIME);
	
	ARM_LEFT_UP;
}

void put_center_right_red(plant_t *plant_control)
{
	ARM_RIGHT_DOWN;
	
	vTaskDelay(GAP_TIME);
	
	move_servo_lsc(&plant_control->servo_right[1],SERVO1_RIGHT_OPEN);
	
	vTaskDelay(GAP_TIME);
	ARM_RIGHT_UP;
}

//������
void get_single(plant_t *plant_control)
{
	ARM_LEFT_DOWN;
	vTaskDelay(GAP_TIME);
	move_servo_lsc(&plant_control->servo_left[2],SERVO2_LEFT_CLOSE);
//	move_servo_lsc(&plant_control->servo_left[1],SERVO1_LEFT_CLOSE);
//	move_servo_lsc(&plant_control->servo_left[2],SERVO2_LEFT_CLOSE);
	
	vTaskDelay(GAP_TIME);
	ARM_LEFT_UP;
}

void put_single(plant_t *plant_control)
{
	ARM_LEFT_DOWN;
	vTaskDelay(GAP_TIME);
	move_servo_lsc(&plant_control->servo_left[2],SERVO2_LEFT_OPEN);
//	move_servo_lsc(&plant_control->servo_left[1],SERVO1_LEFT_CLOSE);
//	move_servo_lsc(&plant_control->servo_left[2],SERVO2_LEFT_CLOSE);
	
	vTaskDelay(GAP_TIME);
	ARM_LEFT_UP;
}

/**
  * @brief          ������ҡ״̬�µİ���ָ��
  * @param[out]     plant_terminal ����ָ��.
  * @retval         none
  */
static void plant_key_handle(plant_t *plant_key)
{
	if(plant_key->game == MAIN)
	{		
			if(plant_key->zone == PLANT_BLUE)
			{
				switch(plant_key->plant_RC->mKey)
				{
					case KEY_L_U:
						plant_key->status = GET_LEFT1;
						break;
					case KEY_L_L:
						plant_key->status = PUT_SIDES_LEFT1;
						break;
					case KEY_L_D:
						plant_key->status = PUT_CENTER_LEFT1;
						break;
					case KEY_R_U:
						plant_key->status = GET_RIGHT1;
						break;
					case KEY_R_L:
						plant_key->status = PUT_SIDES_RIGHT1;
						break;
					case KEY_R_D:
						plant_key->status = PUT_CENTER_RIGHT1;
						break;
				}
			}
			else if(plant_key->zone == PLANT_RED)
			{	
				switch(plant_key->plant_RC->mKey)
				{
					case KEY_L_U:
						plant_key->status = GET_LEFT1_RED;
						break;
					case KEY_L_L:
						plant_key->status = PUT_SIDES_LEFT1_RED;
						break;
					case KEY_L_D:
						plant_key->status = PUT_CENTER_LEFT1_RED;
						break;
					case KEY_R_U:
						plant_key->status = GET_RIGHT1_RED;
						break;
					case KEY_R_L:
						plant_key->status = PUT_SIDES_RIGHT1_RED;
						break;
					case KEY_R_D:
						plant_key->status = PUT_CENTER_RIGHT1_RED;
						break;
				}
			}
	}
	else if(plant_key->game == SINGLE)
	{
		switch(plant_key->plant_RC->mKey)
		{
			case KEY_R_U:
				plant_key->status = GET_SINGLE;
				break;
			case KEY_R_D:
				plant_key->status = PUT_SINGLE;
				break;
		}
	}
/***************************************************************************/
//	if(plant_key->plant_RC->remote.zone == PLANT_BLUE)
//	{
//		if(plant_key->plant_RC->remote.key7 == SINGLE_CLICK)
//		{
//			plant_key->status = GET_LEFT1;
//		}
//		else if(plant_key->plant_RC->remote.key8 == SINGLE_CLICK)
//		{
//			plant_key->status = PUT_SIDES_LEFT1;
//		}
//		else if(plant_key->plant_RC->remote.key10 == SINGLE_CLICK)
//		{
//			plant_key->status = PUT_CENTER_LEFT1;
//		}
//		else if(plant_key->plant_RC->remote.key1 == SINGLE_CLICK)
//		{
//			plant_key->status = GET_RIGHT1;
//		}
//		else if(plant_key->plant_RC->remote.key2 == SINGLE_CLICK)
//		{
//			plant_key->status = PUT_SIDES_RIGHT1;
//		}
//		else if(plant_key->plant_RC->remote.key3 == SINGLE_CLICK)
//		{
//			plant_key->status = PUT_CENTER_RIGHT1;
//		}
//	}
//	else if(plant_key->plant_RC->remote.zone == PLANT_RED)
//	{
//		if(plant_key->plant_RC->remote.key7 == SINGLE_CLICK)
//		{
//			plant_key->status = GET_LEFT1_RED;
//		}
//		else if(plant_key->plant_RC->remote.key8 == SINGLE_CLICK)
//		{
//			plant_key->status = PUT_SIDES_LEFT1_RED;
//		}
//		else if(plant_key->plant_RC->remote.key10 == SINGLE_CLICK)
//		{
//			plant_key->status = PUT_CENTER_LEFT1_RED;
//		}
//		else if(plant_key->plant_RC->remote.key1 == SINGLE_CLICK)
//		{
//			plant_key->status = GET_RIGHT1_RED;
//		}
//		else if(plant_key->plant_RC->remote.key2 == SINGLE_CLICK)
//		{
//			plant_key->status = PUT_SIDES_RIGHT1_RED;
//		}
//		else if(plant_key->plant_RC->remote.key3 == SINGLE_CLICK)
//		{
//			plant_key->status = PUT_CENTER_RIGHT1_RED;
//		}
//	}
}

/**
  * @brief          ȡ�綯������̨�����ÿ���״̬
  * @param[out]     plant_terminal ����ָ��.
  * @retval         none
  */
static void plant_set_terminal(plant_t *plant_terminal)
{
	static uint32_t rx_buff;
	if(plant_terminal->plant_RC->sw[RC_SW_L_CHANNEL] == RC_SW_UP && plant_terminal->plant_RC->sw[RC_SW_R_CHANNEL] == RC_SW_MID)
	{
		plant_terminal->control_mode = PLANT_MANUAL;
		plant_key_handle(plant_terminal);
	}
	else if(plant_terminal->plant_RC->sw[RC_SW_L_CHANNEL] == RC_SW_DOWN )
	{
		plant_terminal->control_mode = PLANT_AUTO;
		
		xQueueReceive(RobotToPlantHandle,&rx_buff,0);
		plant_terminal->status = (plant_status_e)rx_buff;
		rx_buff = PLANT_READY;
	}
	else
	{
		plant_terminal->control_mode = PLANT_CMD_ERROR;
	}
/*********************************************************************************************/
//	static uint32_t rx_buff;
//	if(plant_terminal->plant_RC->remote.sw2 == SW_UP && plant_terminal->plant_RC->remote.sw1 == SW_DOWN)
//	{
//		plant_terminal->control_mode = PLANT_MANUAL;
//		plant_key_handle(plant_terminal);
//	}
//	else if(plant_terminal->plant_RC->remote.sw2 == SW_DOWN )
//	{
//		plant_terminal->control_mode = PLANT_AUTO;
//		if(xQueueReceive(RobotToPlantHandle,&rx_buff,0) == pdTRUE)
//		{
//			plant_terminal->status = (plant_status_e)rx_buff;
//		}
//	}
//	else
//	{
//		plant_terminal->control_mode = PLANT_CMD_ERROR;
//	}
}


/**
  * @brief          ִ��ȡ�綯��
  * @param[out]     plant_control ����ָ��.
  * @retval         none
  */
static void plant_control_loop(plant_t *plant_control)
{
	if(plant_control->control_mode == PLANT_MANUAL || plant_control->control_mode == PLANT_AUTO)
	{
		static uint8_t check_arm_down = 0;
		ARM_LEFT_STRETCH;
		ARM_RIGHT_STRETCH;
		if(check_arm_down == 0)
		{
			ARM_LEFT_DOWN;
			ARM_RIGHT_DOWN;
			check_arm_down = 1;
		}
	}

	uint32_t tx_data = plant_control->status ;	
	switch(plant_control->status)
	{
		case PLANT_READY:
			break; 	 
		case GET_LEFT1: 
		case GET_LEFT2:
		case GET_LEFT1_RED: 
		case GET_LEFT2_RED:
			get_left(plant_control);
			vTaskDelay(GET_TIME);		
			break;
		case GET_RIGHT1:
		case GET_RIGHT2:	
		case GET_RIGHT1_RED:
		case GET_RIGHT2_RED:	
			get_right(plant_control);
			vTaskDelay(PUT_TIME);
			break;
		case PUT_SIDES_LEFT1:
		case PUT_SIDES_LEFT2:
			put_sides_left(plant_control);
			vTaskDelay(PUT_TIME);
			break;
		case PUT_SIDES_RIGHT1:
		case PUT_SIDES_RIGHT2:	
			put_sides_right(plant_control);
			vTaskDelay(GET_TIME);
			break;
		case PUT_CENTER_LEFT1:
		case PUT_CENTER_LEFT2:
			put_center_left(plant_control);
			vTaskDelay(PUT_TIME);
			break;
		case PUT_CENTER_RIGHT1:
		case PUT_CENTER_RIGHT2:	
			put_center_right(plant_control);
			vTaskDelay(PUT_TIME);
			break;
		case PUT_SIDES_LEFT1_RED:
		case PUT_SIDES_LEFT2_RED:
			put_sides_left_red(plant_control);
			vTaskDelay(PUT_TIME);
			break;
		case PUT_SIDES_RIGHT1_RED:
		case PUT_SIDES_RIGHT2_RED:	
			put_sides_right_red(plant_control);
			vTaskDelay(GET_TIME);
			break;
		case PUT_CENTER_LEFT1_RED:
		case PUT_CENTER_LEFT2_RED:
			put_center_left_red(plant_control);
			vTaskDelay(PUT_TIME);
			break;
		case PUT_CENTER_RIGHT1_RED:
		case PUT_CENTER_RIGHT2_RED:
			put_center_right_red(plant_control);
			vTaskDelay(PUT_TIME);
			break;
		case PUT_SINGLE:
			put_single(plant_control);
			break;
		case GET_SINGLE:
			get_single(plant_control);
			break;
		default:
			break;
	}
	if(plant_control->control_mode ==  PLANT_AUTO && plant_control->status != PLANT_READY)
	{
		xQueueSend(PlantToRobotHandle,&tx_data ,0);
		plant_test++;
	}
	plant_control->status = PLANT_READY;	
}


/************************ (C) COPYRIGHT 2024 EPOCH *****END OF FILE****/
