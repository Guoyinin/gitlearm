/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date   
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
//#include "remote_task.h"
#include "cmsis_os.h"
#include "c620.h"
#include "user_lib.h"
#include "queue.h"
#include "stdio.h"
#include "ops_receive.h"
#include "dt35.h"
#include "bsp_rc.h"
/* Exported types ------------------------------------------------------------*/

#define V_SPEED_LIMIT    15//m/s
#define W_SPEED_LIMIT    0.9//rad/s
#define CAR_LENGTH    0.64//m  
#define CAR_WIDTH     0.64//m  
#define Circle_rad 0.785 //45�ȵĻ�����

//����״̬
typedef enum
{
	CHASSIS_MANUAL    = 0,
	CHASSIS_AUTO      = 1,
	CHASSIS_CMD_ERROR = 2,
}chassis_control_e;

//��ҡ�����Զ�״̬
typedef enum
{
	MANUAL_ERROR = -1,
	
	//����ҡ
	MANUAL = 0,
	
	//���Զ�����״̬
	SEMI_AUTO_READY = 1,
	
	//�������볡
	CHASSIS_GET_BALL1    = 15, 
	CHASSIS_SHOOT_BALL1  = 16, 
	CHASSIS_GET_BALL2    = 17, 
	CHASSIS_SHOOT_BALL2  = 18, 
	CHASSIS_GET_BALL3    = 19, 
	CHASSIS_SHOOT_BALL3  = 20,
	CHASSIS_GET_BALL4    = 21, 
	CHASSIS_SHOOT_BALL4  = 22, 
	CHASSIS_GET_BALL5    = 23, 
	CHASSIS_SHOOT_BALL5  = 24, 
	CHASSIS_GET_BALL6    = 25, 
	CHASSIS_SHOOT_BALL6  = 26, 
	CHASSIS_GET_BALL7    = 27, 
	CHASSIS_SHOOT_BALL7  = 28, 
	CHASSIS_GET_BALL8    = 29, 
	CHASSIS_SHOOT_BALL8  = 30, 
	CHASSIS_GET_BALL9    = 31, 
	CHASSIS_SHOOT_BALL9  = 32, 
	CHASSIS_GET_BALL10   = 33, 
	CHASSIS_SHOOT_BALL10 = 34, 
	CHASSIS_GET_BALL11   = 35, 
	CHASSIS_SHOOT_BALL11 = 36, 
	CHASSIS_GET_BALL12   = 37, 
	CHASSIS_SHOOT_BALL12 = 38,
	
	//������볡
	CHASSIS_GET_BALL1_RED    = 115, 
	CHASSIS_SHOOT_BALL1_RED  = 116, 
	CHASSIS_GET_BALL2_RED    = 117, 
	CHASSIS_SHOOT_BALL2_RED  = 118, 
	CHASSIS_GET_BALL3_RED    = 119, 
	CHASSIS_SHOOT_BALL3_RED  = 120,
	CHASSIS_GET_BALL4_RED    = 121, 
	CHASSIS_SHOOT_BALL4_RED  = 122, 
	CHASSIS_GET_BALL5_RED    = 123, 
	CHASSIS_SHOOT_BALL5_RED  = 124, 
	CHASSIS_GET_BALL6_RED    = 125, 
	CHASSIS_SHOOT_BALL6_RED  = 126, 
	CHASSIS_GET_BALL7_RED    = 127, 
	CHASSIS_SHOOT_BALL7_RED  = 128, 
	CHASSIS_GET_BALL8_RED    = 129, 
	CHASSIS_SHOOT_BALL8_RED  = 130, 
	CHASSIS_GET_BALL9_RED    = 131, 
	CHASSIS_SHOOT_BALL9_RED  = 132, 
	CHASSIS_GET_BALL10_RED   = 133, 
	CHASSIS_SHOOT_BALL10_RED = 134, 
	CHASSIS_GET_BALL11_RED   = 135, 
	CHASSIS_SHOOT_BALL11_RED = 136, 
	CHASSIS_GET_BALL12_RED   = 137, 
	CHASSIS_SHOOT_BALL12_RED = 138,
}manual_e;

//ȫ�Զ�״̬
typedef enum
{
	AUTO_READY    = 0,
	
	//һ�����볡
	GET_SEEDLING1 = 1,
	GET_SEEDLING2 = 2,
	PUT_SEEDLING1 = 3,
	PUT_SEEDLING2 = 4,
	PUT_SEEDLING3 = 5,
	PUT_SEEDLING4 = 6,
	GET_SEEDLING3 = 7,
	GET_SEEDLING4 = 8,
	PUT_SEEDLING5 = 9,
	PUT_SEEDLING6 = 10,
	PUT_SEEDLING7 = 11,
	PUT_SEEDLING8 = 12,
	
	
	GOTO_ZONE2    = 13,
	DT35_CORRECT  = 14,
	
	//һ����볡
	GET_SEEDLING1_RED = 101,
	GET_SEEDLING2_RED = 102,
	PUT_SEEDLING1_RED = 103,
	PUT_SEEDLING2_RED = 104,
	PUT_SEEDLING3_RED = 105,
	PUT_SEEDLING4_RED = 106,
	GET_SEEDLING3_RED = 107,
	GET_SEEDLING4_RED = 108,
	PUT_SEEDLING5_RED = 109,
	PUT_SEEDLING6_RED = 110,
	PUT_SEEDLING7_RED = 111,
	PUT_SEEDLING8_RED = 112,
	
	
	GOTO_ZONE2_RED    = 113,
	DT35_CORRECT_RED  = 114,
}auto_e;

//���ֺ�볡�����볡
typedef enum
{
	CHASSIS_BLUE = 0,
	CHASSIS_RED  = 1,
}chassis_court_e;

typedef enum
{
	NULL_CONTROL     = 0,
	CHASSIS_ZERO     = 1,
	CHASSIS_LEFT_90  = 2,
	CHASSIS_RIGHT_90 = 3,
}chassis_yaw_status;


typedef struct
{
  //********************ң����*********************
//  const rc_info1_t *chassis_RC;               //����ʹ�õ�ң����ָ��
  const rc_info_t *chassis_RC;               //����ʹ�õ�ң����ָ��
  //******************�������ӿ���******************
  motor_measure_t motor_vel[4];           //����M3508�������
  fp32 wheel_velocity[4];  	
  PID_TypeDef motor_pid[4];
	PID_TypeDef yaw_correct_pid;
	//PID_TypeDef auto_pid[4];
  //******************�˲���ϵ��*******************
  first_order_filter_type_t chassis_cmd_slow_set_vx;  //ʹ��һ�׵�ͨ�˲������趨ֵ
	first_order_filter_type_t chassis_cmd_slow_set_vy;  //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_wz;  //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_zero;  //ʹ��һ�׵�ͨ�˲������趨ֵ
  //******************����ƽ�ơ���ת�ٶ��趨(������ϵ)******************	
	fp32 car_dis_set;//����ƽ�Ʒ���Ƕȣ���������ϵ����λ���Ƕ�
  fp32 car_vel_set;//���������ٶȣ���������ϵ��ң��������ϵ��
  fp32 car_yaw_set;//���̺����
  
  //*******************�����ٶ��趨ԭʼֵ(ң��������)*******************
  fp32 vx_set;                      //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	
	  //******************ops����λ����Ϣ******************
  ops_t ops;
	
  //******************����ƽ������******************
  fp32 car_max_speed;  //����ٶ� ��λm/s
  fp32 car_min_speed;  //����ٶ� ��λm/s
  fp32 wz_max_speed;  //����ٶ� ��λm/s
  fp32 wz_min_speed;  //����ٶ� ��λm/s

	/******************���Զ�******************/
	float target;                  //��¼�����ĸ���
	fp32 pos_x_target;             //λ�û�Ŀ��x
	fp32 pos_y_target;             //λ�û�Ŀ��y
	PID_TypeDef yaw_pid; 
	PID_TypeDef pos_x_pid;
	PID_TypeDef pos_y_pid;
	
	uint8_t zone;
	uint8_t game;
	
	chassis_control_e  control_mode;  //�Ƿ���ҡ
	manual_e        manual_status; //���Զ�����״̬
	auto_e          auto_status;   //ȫ�Զ�����״̬
	
	uint8_t zero_set;
	//chassis_court_e court;         
} chassis_move_t;



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//���ҵ�ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//ǰ���ң����ͨ������
#define CHASSIS_Y_CHANNEL 2
//��ת��ң����ͨ������
#define CHASSIS_WZ_CHANNEL 0
//�����˶�����ͨ������
#define CHASSIS_SLOW_CHANNEL KEY8_PRESS

//TODO: �������ʵ��λ
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���660*0.005=3.30
#define CHASSIS_VX_RC_SEN 0.002f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.002f
//ң������תҡ��/���֣�max 660��ת���ɳ����ٶȣ�m/s���ı���
#define CHASSIS_WZ_RC_SEN -0.001f 
//�������ٱ���
#define CHASSIS_SLOW 0.3f

//Ӱ��һ�׵�ͨ�˲���ϵ�������ϵ��Խ���˲�Ч��Խ���ԣ�Խƽ����������Խ��
#define CHASSIS_ACCEL_VEL 0.01f
#define CHASSIS_ACCEL_YAW 0.0f //δʹ��


//ҡ������
#define CHASSIS_RC_XY_DEADLINE 80
#define CHASSIS_RC_WZ_DEADLINE 300




//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//�����������Ƶ��
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.0f
//����ƫ����PID����
#define CHASSIS_YAW_PID_KP 0.01f
#define CHASSIS_YAW_PID_KI 0.0f
#define CHASSIS_YAW_PID_KD 0.0f
#define CHASSIS_YAW_PID_MAX_OUT 2.0f
#define CHASSIS_YAW_PID_MAX_IOUT 5.0f
#define CHASSIS_YAW_PID_DEADBAND 2.0f

/* Exported functions ------------------------------------------------------- */

/**
  * @brief          ��������
  * @param[in]      argument: ��
  * @retval         none
  */
void chassis_task(void const * argument);


#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

