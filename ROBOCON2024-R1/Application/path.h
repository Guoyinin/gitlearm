/**
  ******************************************************************************
  * @file    path.h
  * @author  
  * @version 
  * @date   
  * @brief   This file contains the headers of path.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PATH_H_
#define __PATH_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "chassis_task.h"

/* Exported types ------------------------------------------------------------*/

typedef struct
{
	float x;
	float y;
}position_t;
/**
 * @brief 位置信息，用于自动寻路
 */
typedef struct
{
  position_t    position;   //位置
	float         direction; 	// 航向角（角度制）
	float         vel;        //跑向这个点的速度
	//float      		wz;					//旋转分量速度
}pose_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//路径长度
#define GET_SEEDLING1_PATH_LEN 4
#define GET_SEEDLING2_PATH_LEN 4

#define PUT_SEEDLING1_PATH_LEN 3
#define PUT_SEEDLING2_PATH_LEN 3
//#define PUT_SEEDLING3_PATH_LEN 4
#define PUT_SEEDLING3_PATH_LEN 3
#define PUT_SEEDLING4_PATH_LEN 3

#define GET_SEEDLING3_PATH_LEN 2
#define GET_SEEDLING4_PATH_LEN 4

#define PUT_SEEDLING5_PATH_LEN 2
#define PUT_SEEDLING6_PATH_LEN 3
#define PUT_SEEDLING7_PATH_LEN 3
#define PUT_SEEDLING8_PATH_LEN 3

#define GET_BALL1_PATH_LEN     2
#define GET_BALL2_PATH_LEN     2
#define GET_BALL3_PATH_LEN     2
#define GET_BALL4_PATH_LEN     2
#define GET_BALL5_PATH_LEN     2
#define GET_BALL6_PATH_LEN     2
#define GET_BALL7_PATH_LEN     2
#define GET_BALL8_PATH_LEN     2
#define GET_BALL9_PATH_LEN     2
#define GET_BALL10_PATH_LEN    2
#define GET_BALL11_PATH_LEN    2
#define GET_BALL12_PATH_LEN    2

#define GOTO_ZONE2_PATH_LEN  4 

#define SHOOT_BALL1_PATH_LEN   1
#define SHOOT_BALL2_PATH_LEN   1
#define SHOOT_BALL3_PATH_LEN   1
#define SHOOT_BALL4_PATH_LEN   1
#define SHOOT_BALL5_PATH_LEN   1
#define SHOOT_BALL6_PATH_LEN   1
#define SHOOT_BALL7_PATH_LEN   2
#define SHOOT_BALL8_PATH_LEN   2
#define SHOOT_BALL9_PATH_LEN   2
#define SHOOT_BALL10_PATH_LEN  2
#define SHOOT_BALL11_PATH_LEN  2
#define SHOOT_BALL12_PATH_LEN  2

//规划的速度
#define MAX_PLAN_VEL 	1500
#define MAX_PLAN_WZ 	200

//#define X_PID_MIN_DEADBAND  110
//#define Y_PID_MIN_DEADBAND  110 
//#define Z_PID_MIN_DEADBAND  110

//路径的死区，小于该值认为跑到点
#define PATH_X_DEAD     5.0f
#define PATH_Y_DEAD     5.0f
#define PATH_W_DEAD     2.0f
#define PATH_SPEED_DEAD 0.5f

//机械设计、场地的固定参数
#define ARM_LEN                 125	              //机械臂长度
#define CHASSIS_LEN             620               //底盘的边长
#define CLAMP_LEN               120               //夹爪距离车边的长度
#define CHASSIS_LEN_HALF        (CHASSIS_LEN/2)   //底盘半边长，该距离为action到底盘边框的理论长度(理论上认为action在正中心)
#define ZONE2_BALL_GAP          500.0f            //二区两个球之间的距离
#define ZONE2_BALL_GAP_HALF     (ZONE2_BALL_GAP/2)
#define SLOPE_BEVEL_SIDE        1004.99f          //斜坡的斜边  
#define SLOPE_RIGHT_ANGLE_SIDE  1000.0f           //斜坡的直角边
#define SLOPE_DELTA             (SLOPE_BEVEL_SIDE - SLOPE_RIGHT_ANGLE_SIDE)
#define MAP_Y_BOARD             7000.0f           //二区挡板的Y轴坐标

//机械安装的偏差
#define ACTION_ERROR        277.0f
#define SERVO_LEFT_ERROR    20.0f
#define SERVO_RIGHT_ERROR   25.0f

//地图偏差
#define X_PUT_SEEDING1_MAP_ERROR 0.0f
#define Y_PUT_SEEDING1_MAP_ERROR 0.0f

#define X_ZONE2_MAP_ERROR        100.0f 

//向过渡点移动的距离
#define CAR_X_MOVE_LOW_DISTANCE    400.0f
#define CAR_Y_MOVE_LOW_DISTANCE    400.0f

#define CAR_X_MOVE_SMALL_DISTANCE    200.0f
#define CAR_Y_MOVE_SMALL_DISTANCE    200.0f

//action到场地的距离 
#define CAR_X_OFFSET    1290.0f
#define CAR_Y_OFFSET    310.0f

//实际在地图中的位置
#define MAP_X_START  1290.0f
#define MAP_Y_START  310.0f

#define CAR_X_START    -(MAP_X_START - CAR_X_OFFSET)
#define CAR_Y_START     (MAP_Y_START - CAR_Y_OFFSET) 

//目标点到场地边的距离
//一区
#define MAP_X_GET_SEEDLING1  3250.0f
#define MAP_Y_GET_SEEDLING1  125.0f
#define MAP_X_GET_SEEDLING2  2500.0f
#define MAP_Y_GET_SEEDLING2  125.0f

//#define MAP_X_PUT_SEEDLING1  4875.0f
//#define MAP_Y_PUT_SEEDLING1  2500.0f
//#define MAP_X_PUT_SEEDLING2  4375.0f
//#define MAP_Y_PUT_SEEDLING2  2750.0f

#define MAP_X_PUT_SEEDLING1  2625.0f
#define MAP_Y_PUT_SEEDLING1  2750.0f
#define MAP_X_PUT_SEEDLING2  3375.0f
#define MAP_Y_PUT_SEEDLING2  2750.0f

//#define MAP_X_PUT_SEEDLING1  4625.0f
//#define MAP_Y_PUT_SEEDLING1  2750.0f
//#define MAP_X_PUT_SEEDLING2  4375.0f
//#define MAP_Y_PUT_SEEDLING2  2750.0f
//#define MAP_X_PUT_SEEDLING3  4375.0f
//#define MAP_Y_PUT_SEEDLING3  2250.0f
//#define MAP_X_PUT_SEEDLING4  3875.0f
//#define MAP_Y_PUT_SEEDLING4  2500.0f

#define MAP_X_PUT_SEEDLING3  3875.0f
#define MAP_Y_PUT_SEEDLING3  2750.0f
#define MAP_X_PUT_SEEDLING4  4625.0f
#define MAP_Y_PUT_SEEDLING4  2750.0f

//#define MAP_X_PUT_SEEDLING3  4125.0f
//#define MAP_Y_PUT_SEEDLING3  2750.0f
//#define MAP_X_PUT_SEEDLING4  3125.0f
//#define MAP_Y_PUT_SEEDLING4  2750.0f

#define MAP_X_GET_SEEDLING3  4000.0f
#define MAP_Y_GET_SEEDLING3  125.0f
#define MAP_X_GET_SEEDLING4  4750.0f
#define MAP_Y_GET_SEEDLING4  125.0f

//#define MAP_X_PUT_SEEDLING5  3375.0f
//#define MAP_Y_PUT_SEEDLING5  2500.0f
//#define MAP_X_PUT_SEEDLING6  2875.0f
//#define MAP_Y_PUT_SEEDLING6  2750.0f

#define MAP_X_PUT_SEEDLING5  4625.0f
#define MAP_Y_PUT_SEEDLING5  2250.0f
#define MAP_X_PUT_SEEDLING6  3875.0f
#define MAP_Y_PUT_SEEDLING6  2250.0f

#define MAP_X_PUT_SEEDLING7  3375.0f
#define MAP_Y_PUT_SEEDLING7  2250.0f
#define MAP_X_PUT_SEEDLING8  2625.0f
#define MAP_Y_PUT_SEEDLING8  2250.0f

//纠偏
#define MAP_X_BEFORE_SLOPE   600.0f
#define MAP_Y_BEFORE_SLOPE   2000.0f
#define MAP_X_DT35_CORRECT   1290.0f
#define MAP_Y_DT35_CORRECT   5540.0f

//二区
#define MAP_X_GET_BALL1  4875.0f
#define MAP_Y_GET_BALL1  5750.0f
#define MAP_X_GET_BALL2  4375.0f
#define MAP_Y_GET_BALL2  5750.0f
#define MAP_X_GET_BALL3  3875.0f
#define MAP_Y_GET_BALL3  5750.0f
#define MAP_X_GET_BALL4  3375.0f
#define MAP_Y_GET_BALL4  5750.0f
#define MAP_X_GET_BALL5  2875.0f
#define MAP_Y_GET_BALL5  5750.0f
#define MAP_X_GET_BALL6  2375.0f
#define MAP_Y_GET_BALL6  5750.0f
							
#define MAP_X_GET_BALL7  4875.0f
#define MAP_Y_GET_BALL7  5250.0f
#define MAP_X_GET_BALL8  4375.0f
#define MAP_Y_GET_BALL8  5250.0f
#define MAP_X_GET_BALL9  3875.0f
#define MAP_Y_GET_BALL9  5250.0f
#define MAP_X_GET_BALL10 3375.0f
#define MAP_Y_GET_BALL10 5250.0f
#define MAP_X_GET_BALL11 2875.0f
#define MAP_Y_GET_BALL11 5250.0f
#define MAP_X_GET_BALL12 2375.0f
#define MAP_Y_GET_BALL12 5250.0f

#define MAP_X_ZONE2_TEST 5875.0f
#define MAP_Y_ZONE2_TEST 5000.0f

//路径中代入的坐标
//一区
#define CAR_X_GET_SEEDLING1   -(MAP_X_GET_SEEDLING1 - CAR_X_OFFSET)
#define CAR_Y_GET_SEEDLING1    (MAP_Y_GET_SEEDLING1 - CAR_Y_OFFSET + ARM_LEN + CHASSIS_LEN_HALF) 
#define CAR_X_GET_SEEDLING2   -(MAP_X_GET_SEEDLING2 - CAR_X_OFFSET)
#define CAR_Y_GET_SEEDLING2    (MAP_Y_GET_SEEDLING2 - CAR_Y_OFFSET + ARM_LEN + CHASSIS_LEN_HALF) 

#define CAR_X_PUT_SEEDLING1   -(MAP_X_PUT_SEEDLING1 - CAR_X_OFFSET)
#define CAR_Y_PUT_SEEDLING1    (MAP_Y_PUT_SEEDLING1 - CAR_Y_OFFSET + ARM_LEN + CHASSIS_LEN_HALF) 
#define CAR_X_PUT_SEEDLING2   -(MAP_X_PUT_SEEDLING2 - CAR_X_OFFSET)
#define CAR_Y_PUT_SEEDLING2    (MAP_Y_PUT_SEEDLING2 - CAR_Y_OFFSET + ARM_LEN + CHASSIS_LEN_HALF) 

//#define CAR_X_PUT_SEEDLING3   -(MAP_X_PUT_SEEDLING3 - CAR_X_OFFSET - ARM_LEN - CHASSIS_LEN_HALF)
//#define CAR_Y_PUT_SEEDLING3    (MAP_Y_PUT_SEEDLING3 - CAR_Y_OFFSET) 
//#define CAR_X_PUT_SEEDLING4   -(MAP_X_PUT_SEEDLING4 - CAR_X_OFFSET - ARM_LEN - CHASSIS_LEN_HALF)
//#define CAR_Y_PUT_SEEDLING4    (MAP_Y_PUT_SEEDLING4 - CAR_Y_OFFSET)

#define CAR_X_PUT_SEEDLING3   -(MAP_X_PUT_SEEDLING3 - CAR_X_OFFSET )
#define CAR_Y_PUT_SEEDLING3    (MAP_Y_PUT_SEEDLING3 - CAR_Y_OFFSET - ARM_LEN - CHASSIS_LEN_HALF) 
#define CAR_X_PUT_SEEDLING4   -(MAP_X_PUT_SEEDLING4 - CAR_X_OFFSET)
#define CAR_Y_PUT_SEEDLING4    (MAP_Y_PUT_SEEDLING4 - CAR_Y_OFFSET - ARM_LEN - CHASSIS_LEN_HALF)

#define CAR_X_GET_SEEDLING3   -(MAP_X_GET_SEEDLING3 - CAR_X_OFFSET)
#define CAR_Y_GET_SEEDLING3    (MAP_Y_GET_SEEDLING3 - CAR_Y_OFFSET + ARM_LEN + CHASSIS_LEN_HALF) 
#define CAR_X_GET_SEEDLING4   -(MAP_X_GET_SEEDLING4 - CAR_X_OFFSET)
#define CAR_Y_GET_SEEDLING4    (MAP_Y_GET_SEEDLING4 - CAR_Y_OFFSET + ARM_LEN + CHASSIS_LEN_HALF) 

#define CAR_X_PUT_SEEDLING5   -(MAP_X_PUT_SEEDLING5 - CAR_X_OFFSET)
#define CAR_Y_PUT_SEEDLING5    (MAP_Y_PUT_SEEDLING5 - CAR_Y_OFFSET - ARM_LEN - CHASSIS_LEN_HALF) 
#define CAR_X_PUT_SEEDLING6   -(MAP_X_PUT_SEEDLING6 - CAR_X_OFFSET)
#define CAR_Y_PUT_SEEDLING6    (MAP_Y_PUT_SEEDLING6 - CAR_Y_OFFSET - ARM_LEN - CHASSIS_LEN_HALF) 
#define CAR_X_PUT_SEEDLING7   -(MAP_X_PUT_SEEDLING7 - CAR_X_OFFSET)
#define CAR_Y_PUT_SEEDLING7    (MAP_Y_PUT_SEEDLING7 - CAR_Y_OFFSET - ARM_LEN - CHASSIS_LEN_HALF) 
#define CAR_X_PUT_SEEDLING8   -(MAP_X_PUT_SEEDLING8 - CAR_X_OFFSET)
#define CAR_Y_PUT_SEEDLING8    (MAP_Y_PUT_SEEDLING8 - CAR_Y_OFFSET - ARM_LEN - CHASSIS_LEN_HALF)

//纠偏
#define CAR_X_BEFORE_SLOPE    -(MAP_X_BEFORE_SLOPE - CAR_X_OFFSET)
#define CAR_Y_BEFORE_SLOPE     (MAP_Y_BEFORE_SLOPE - CAR_Y_OFFSET - CHASSIS_LEN)
#define CAR_X_DT35_CORRECT    -(MAP_X_DT35_CORRECT - CAR_X_OFFSET) 
#define CAR_Y_DT35_CORRECT     (MAP_Y_DT35_CORRECT - CAR_Y_OFFSET - CHASSIS_LEN_HALF)

//二区
#define CAR_X_GET_BALL1       -(MAP_X_GET_BALL1  - CAR_X_OFFSET)
#define CAR_Y_GET_BALL1        (MAP_Y_GET_BALL1  - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)
#define CAR_X_GET_BALL2       -(MAP_X_GET_BALL2  - CAR_X_OFFSET)
#define CAR_Y_GET_BALL2        (MAP_Y_GET_BALL2  - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)
#define CAR_X_GET_BALL3       -(MAP_X_GET_BALL3  - CAR_X_OFFSET)
#define CAR_Y_GET_BALL3        (MAP_Y_GET_BALL3  - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)
#define CAR_X_GET_BALL4       -(MAP_X_GET_BALL4  - CAR_X_OFFSET)
#define CAR_Y_GET_BALL4        (MAP_Y_GET_BALL4  - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)
#define CAR_X_GET_BALL5       -(MAP_X_GET_BALL5  - CAR_X_OFFSET)
#define CAR_Y_GET_BALL5        (MAP_Y_GET_BALL5  - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)
#define CAR_X_GET_BALL6       -(MAP_X_GET_BALL6  - CAR_X_OFFSET)
#define CAR_Y_GET_BALL6        (MAP_Y_GET_BALL6  - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)

#define CAR_X_GET_BALL7       -(MAP_X_GET_BALL7  - CAR_X_OFFSET)
#define CAR_Y_GET_BALL7        (MAP_Y_GET_BALL7  - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)
#define CAR_X_GET_BALL8       -(MAP_X_GET_BALL8  - CAR_X_OFFSET)
#define CAR_Y_GET_BALL8        (MAP_Y_GET_BALL8  - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)
#define CAR_X_GET_BALL9       -(MAP_X_GET_BALL9  - CAR_X_OFFSET)
#define CAR_Y_GET_BALL9        (MAP_Y_GET_BALL9  - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)
#define CAR_X_GET_BALL10      -(MAP_X_GET_BALL10 - CAR_X_OFFSET)
#define CAR_Y_GET_BALL10       (MAP_Y_GET_BALL10 - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)
#define CAR_X_GET_BALL11      -(MAP_X_GET_BALL11 - CAR_X_OFFSET)
#define CAR_Y_GET_BALL11       (MAP_Y_GET_BALL11 - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)
#define CAR_X_GET_BALL12      -(MAP_X_GET_BALL12 - CAR_X_OFFSET)
#define CAR_Y_GET_BALL12       (MAP_Y_GET_BALL12 - CAR_Y_OFFSET + CLAMP_LEN + CHASSIS_LEN_HALF)

#define CAR_X_SHOOT_BALL1       CAR_X_GET_BALL1
#define CAR_Y_SHOOT_BALL1       CAR_Y_GET_BALL1
#define CAR_X_SHOOT_BALL2       CAR_X_GET_BALL2
#define CAR_Y_SHOOT_BALL2       CAR_Y_GET_BALL2
#define CAR_X_SHOOT_BALL3       CAR_X_GET_BALL3
#define CAR_Y_SHOOT_BALL3       CAR_Y_GET_BALL3
#define CAR_X_SHOOT_BALL4       CAR_X_GET_BALL4
#define CAR_Y_SHOOT_BALL4       CAR_Y_GET_BALL4
#define CAR_X_SHOOT_BALL5       CAR_X_GET_BALL5
#define CAR_Y_SHOOT_BALL5       CAR_Y_GET_BALL5
#define CAR_X_SHOOT_BALL6       CAR_X_GET_BALL6
#define CAR_Y_SHOOT_BALL6       CAR_Y_GET_BALL6
							
#define CAR_X_SHOOT_BALL7       CAR_X_SHOOT_BALL1
#define CAR_Y_SHOOT_BALL7       CAR_Y_SHOOT_BALL1
#define CAR_X_SHOOT_BALL8       CAR_X_SHOOT_BALL2
#define CAR_Y_SHOOT_BALL8       CAR_Y_SHOOT_BALL2
#define CAR_X_SHOOT_BALL9       CAR_X_SHOOT_BALL3
#define CAR_Y_SHOOT_BALL9       CAR_Y_SHOOT_BALL3
#define CAR_X_SHOOT_BALL10      CAR_X_SHOOT_BALL4
#define CAR_Y_SHOOT_BALL10      CAR_Y_SHOOT_BALL4
#define CAR_X_SHOOT_BALL11      CAR_X_SHOOT_BALL5
#define CAR_Y_SHOOT_BALL11      CAR_Y_SHOOT_BALL5
#define CAR_X_SHOOT_BALL12      CAR_X_SHOOT_BALL6
#define CAR_Y_SHOOT_BALL12      CAR_Y_SHOOT_BALL6

#define CAR_X_ZONE2_TEST      -(MAP_X_ZONE2_TEST - CAR_X_OFFSET             - CHASSIS_LEN_HALF)
#define CAR_Y_ZONE2_TEST       (MAP_Y_ZONE2_TEST - CAR_Y_OFFSET             + CHASSIS_LEN_HALF)  


//红半场X轴区相反数,Y轴不变
#define CAR_X_GET_SEEDLING1_RED   -CAR_X_GET_SEEDLING1  
#define CAR_X_GET_SEEDLING2_RED   -CAR_X_GET_SEEDLING2  
			                           
#define CAR_X_PUT_SEEDLING1_RED   -CAR_X_PUT_SEEDLING1  
#define CAR_X_PUT_SEEDLING2_RED   -CAR_X_PUT_SEEDLING2  
#define CAR_X_PUT_SEEDLING3_RED   -CAR_X_PUT_SEEDLING3  
#define CAR_X_PUT_SEEDLING4_RED   -CAR_X_PUT_SEEDLING4  
			                           
#define CAR_X_GET_SEEDLING3_RED   -CAR_X_GET_SEEDLING3  
#define CAR_X_GET_SEEDLING4_RED   -CAR_X_GET_SEEDLING4  
			                         
#define CAR_X_PUT_SEEDLING5_RED   -CAR_X_PUT_SEEDLING5  
#define CAR_X_PUT_SEEDLING6_RED   -CAR_X_PUT_SEEDLING6  
#define CAR_X_PUT_SEEDLING7_RED   -CAR_X_PUT_SEEDLING7  
#define CAR_X_PUT_SEEDLING8_RED   -CAR_X_PUT_SEEDLING8  
			                            
//纠偏                            
#define CAR_X_BEFORE_SLOPE_RED    -CAR_X_BEFORE_SLOPE   
#define CAR_X_DT35_CORRECT_RED    -CAR_X_DT35_CORRECT   
			                            
//二区                            
#define CAR_X_GET_BALL1_RED       -CAR_X_GET_BALL1      
#define CAR_X_GET_BALL2_RED       -CAR_X_GET_BALL2      
#define CAR_X_GET_BALL3_RED       -CAR_X_GET_BALL3      
#define CAR_X_GET_BALL4_RED       -CAR_X_GET_BALL4      
#define CAR_X_GET_BALL5_RED       -CAR_X_GET_BALL5      
#define CAR_X_GET_BALL6_RED       -CAR_X_GET_BALL6      
			                    
#define CAR_X_GET_BALL7_RED       -CAR_X_GET_BALL7      
#define CAR_X_GET_BALL8_RED       -CAR_X_GET_BALL8      
#define CAR_X_GET_BALL9_RED       -CAR_X_GET_BALL9      
#define CAR_X_GET_BALL10_RED       -CAR_X_GET_BALL10     
#define CAR_X_GET_BALL11_RED       -CAR_X_GET_BALL11     
#define CAR_X_GET_BALL12_RED       -CAR_X_GET_BALL12     
			                            
#define CAR_X_SHOOT_BALL1_RED      -CAR_X_SHOOT_BALL1    
#define CAR_X_SHOOT_BALL2_RED      -CAR_X_SHOOT_BALL2    
#define CAR_X_SHOOT_BALL3_RED      -CAR_X_SHOOT_BALL3    
#define CAR_X_SHOOT_BALL4_RED      -CAR_X_SHOOT_BALL4    
#define CAR_X_SHOOT_BALL5_RED      -CAR_X_SHOOT_BALL5    
#define CAR_X_SHOOT_BALL6_RED      -CAR_X_SHOOT_BALL6    
						                      
#define CAR_X_SHOOT_BALL7_RED      -CAR_X_SHOOT_BALL7    
#define CAR_X_SHOOT_BALL8_RED      -CAR_X_SHOOT_BALL8    
#define CAR_X_SHOOT_BALL9_RED      -CAR_X_SHOOT_BALL9    
#define CAR_X_SHOOT_BALL10_RED     -CAR_X_SHOOT_BALL10   
#define CAR_X_SHOOT_BALL11_RED     -CAR_X_SHOOT_BALL11   
#define CAR_X_SHOOT_BALL12_RED     -CAR_X_SHOOT_BALL12   
			                            
#define CAR_X_ZONE2_TEST_RED       -CAR_X_ZONE2_TEST        

//二区发射时的航向角
#define CAR_YAW_SHOOT1         31.6f  
#define CAR_YAW_SHOOT2         27.6f     
#define CAR_YAW_SHOOT3         25.0f     
#define CAR_YAW_SHOOT4         16.53f     
#define CAR_YAW_SHOOT5         18.56f     
#define CAR_YAW_SHOOT6         14.15f     

#define CAR_YAW_SHOOT7         CAR_YAW_SHOOT1
#define CAR_YAW_SHOOT8         CAR_YAW_SHOOT2
#define CAR_YAW_SHOOT9         CAR_YAW_SHOOT3
#define CAR_YAW_SHOOT10        CAR_YAW_SHOOT4
#define CAR_YAW_SHOOT11        CAR_YAW_SHOOT5
#define CAR_YAW_SHOOT12        CAR_YAW_SHOOT6

/* Exported functions ------------------------------------------------------- */

uint8_t path_flow_positon(chassis_move_t *chassis_flow, pose_t *path, uint8_t path_len);
//uint8_t path_flow_speed(chassis_move_t *chassis_flow,pose_t *path,uint8_t path_len);
//uint8_t point_flow_positon(chassis_move_t *chassis_flow, pose_t target_point);

#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

