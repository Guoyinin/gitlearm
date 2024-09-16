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
#define Circle_rad 0.785 //45度的弧度制

//控制状态
typedef enum
{
	CHASSIS_MANUAL    = 0,
	CHASSIS_AUTO      = 1,
	CHASSIS_CMD_ERROR = 2,
}chassis_control_e;

//手摇、半自动状态
typedef enum
{
	MANUAL_ERROR = -1,
	
	//纯手摇
	MANUAL = 0,
	
	//半自动就绪状态
	SEMI_AUTO_READY = 1,
	
	//二区蓝半场
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
	
	//二区红半场
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

//全自动状态
typedef enum
{
	AUTO_READY    = 0,
	
	//一区蓝半场
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
	
	//一区红半场
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

//区分红半场和蓝半场
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
  //********************遥控器*********************
//  const rc_info1_t *chassis_RC;               //底盘使用的遥控器指针
  const rc_info_t *chassis_RC;               //底盘使用的遥控器指针
  //******************底盘轮子控制******************
  motor_measure_t motor_vel[4];           //底盘M3508电机数据
  fp32 wheel_velocity[4];  	
  PID_TypeDef motor_pid[4];
	PID_TypeDef yaw_correct_pid;
	//PID_TypeDef auto_pid[4];
  //******************滤波器系数*******************
  first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓设定值
	first_order_filter_type_t chassis_cmd_slow_set_vy;  //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_wz;  //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_zero;  //使用一阶低通滤波减缓设定值
  //******************底盘平移、旋转速度设定(极坐标系)******************	
	fp32 car_dis_set;//车体平移方向角度（世界坐标系）单位：角度
  fp32 car_vel_set;//车体总体速度（世界坐标系、遥控器坐标系）
  fp32 car_yaw_set;//底盘航向角
  
  //*******************底盘速度设定原始值(遥控器输入)*******************
  fp32 vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s
	
	  //******************ops底盘位置信息******************
  ops_t ops;
	
  //******************底盘平移限速******************
  fp32 car_max_speed;  //最大速度 单位m/s
  fp32 car_min_speed;  //最大速度 单位m/s
  fp32 wz_max_speed;  //最大速度 单位m/s
  fp32 wz_min_speed;  //最大速度 单位m/s

	/******************半自动******************/
	float target;                  //记录跑向哪个点
	fp32 pos_x_target;             //位置环目标x
	fp32 pos_y_target;             //位置环目标y
	PID_TypeDef yaw_pid; 
	PID_TypeDef pos_x_pid;
	PID_TypeDef pos_y_pid;
	
	uint8_t zone;
	uint8_t game;
	
	chassis_control_e  control_mode;  //是否手摇
	manual_e        manual_status; //半自动控制状态
	auto_e          auto_status;   //全自动控制状态
	
	uint8_t zero_set;
	//chassis_court_e court;         
} chassis_move_t;



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//左右的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//旋转的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 0
//减速运动按键通道号码
#define CHASSIS_SLOW_CHANNEL KEY8_PRESS

//TODO: 换算成现实单位
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例660*0.005=3.30
#define CHASSIS_VX_RC_SEN 0.002f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.002f
//遥控器旋转摇杆/拨轮（max 660）转化成车体速度（m/s）的比例
#define CHASSIS_WZ_RC_SEN -0.001f 
//底盘慢速比例
#define CHASSIS_SLOW 0.3f

//影响一阶低通滤波的系数，这个系数越大，滤波效果越明显（越平滑），加速越慢
#define CHASSIS_ACCEL_VEL 0.01f
#define CHASSIS_ACCEL_YAW 0.0f //未使用


//摇杆死区
#define CHASSIS_RC_XY_DEADLINE 80
#define CHASSIS_RC_WZ_DEADLINE 300




//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.0f
//底盘偏航角PID参数
#define CHASSIS_YAW_PID_KP 0.01f
#define CHASSIS_YAW_PID_KI 0.0f
#define CHASSIS_YAW_PID_KD 0.0f
#define CHASSIS_YAW_PID_MAX_OUT 2.0f
#define CHASSIS_YAW_PID_MAX_IOUT 5.0f
#define CHASSIS_YAW_PID_DEADBAND 2.0f

/* Exported functions ------------------------------------------------------- */

/**
  * @brief          底盘任务
  * @param[in]      argument: 空
  * @retval         none
  */
void chassis_task(void const * argument);


#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

