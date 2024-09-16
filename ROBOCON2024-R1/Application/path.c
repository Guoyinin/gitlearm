/**
  ******************************************************************************
  * @file     path.c
  * @author   
  * @version  
  * @date     
  * @brief    
  ******************************************************************************
  * @attention
  * 注意事项
  *
  *
  * 
  ******************************************************************************
  */ 

	
/* Includes -------------------------------------------------------------------*/
#include "path.h"
#include "math.h"
#include "ops_receive.h"
#include "chassis_task.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/

#define ABS(x) ((x) > 0 ? (x) : -(x))

/* Private  variables ---------------------------------------------------------*/
extern motor_measure_t motor_chassis[4];
extern int16_t ave_speed;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/************************************蓝半场****************************************/
//一区
//一区

pose_t get_seedling1_path[GET_SEEDLING1_PATH_LEN]=
{
	{CAR_X_START                       ,CAR_Y_START                                 ,0.0f  ,   0.0f},
	{CAR_X_START                          ,CAR_Y_START + CAR_Y_MOVE_LOW_DISTANCE       ,0.0f  ,   MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING1 - ACTION_ERROR,CAR_Y_START + CAR_Y_MOVE_LOW_DISTANCE       ,-90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING1 - ACTION_ERROR-35.0f,CAR_Y_GET_SEEDLING1 - ACTION_ERROR+5.0f 	,-90.0f,  MAX_PLAN_VEL},
};

pose_t get_seedling2_path[GET_SEEDLING2_PATH_LEN]=
{
	{CAR_X_GET_SEEDLING1 - ACTION_ERROR -35.0f                  , CAR_Y_GET_SEEDLING1 - ACTION_ERROR+5.0f  ,													-90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING1 - ACTION_ERROR                   , CAR_Y_GET_SEEDLING1 + CAR_Y_MOVE_LOW_DISTANCE- ACTION_ERROR ,	-90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING2 + ACTION_ERROR -SERVO_RIGHT_ERROR, CAR_Y_GET_SEEDLING1 + CAR_Y_MOVE_LOW_DISTANCE- ACTION_ERROR , 90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING2 + ACTION_ERROR -SERVO_RIGHT_ERROR-50.0f, CAR_Y_GET_SEEDLING2 - ACTION_ERROR    + 10.0f ,												90.0f, MAX_PLAN_VEL},
};

pose_t put_seedling1_path[PUT_SEEDLING1_PATH_LEN]=
{
	{CAR_X_GET_SEEDLING2 + ACTION_ERROR -SERVO_RIGHT_ERROR -50.0f           , CAR_Y_GET_SEEDLING2 - ACTION_ERROR  + 10.0f                  , 90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING2 + ACTION_ERROR -SERVO_RIGHT_ERROR                  , CAR_Y_PUT_SEEDLING1 - ACTION_ERROR                           , 90.0f  , MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING1 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR -110.0f  , CAR_Y_PUT_SEEDLING1 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR+15.0f, 90.0f  , MAX_PLAN_VEL},
};

pose_t put_seedling2_path[PUT_SEEDLING2_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING1 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR -110.0f , CAR_Y_PUT_SEEDLING1 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR +15.0f                                , 90.0f  , MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING2 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR        , CAR_Y_PUT_SEEDLING1 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR + CAR_Y_MOVE_LOW_DISTANCE-100.0f, 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING2 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR-110.0f , CAR_Y_PUT_SEEDLING2 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR +15.0f                                , 90.0f, MAX_PLAN_VEL},
};                                                       

pose_t put_seedling3_path[PUT_SEEDLING3_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING2 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR-110.0f , CAR_Y_PUT_SEEDLING2 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR +80.0f                                , 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING3 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR-110.0f , CAR_Y_PUT_SEEDLING2 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR +80.0f                                , 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING3 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR - 110.0f, CAR_Y_PUT_SEEDLING3 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR - 20.0f   , 90.0f, MAX_PLAN_VEL},
//	{CAR_X_PUT_SEEDLING3                                      + X_PUT_SEEDING1_MAP_ERROR , CAR_Y_PUT_SEEDLING3 + Y_PUT_SEEDING1_MAP_ERROR - 2*ACTION_ERROR + SERVO_RIGHT_ERROR,           -180.0f, MAX_PLAN_VEL},
};                                                          

pose_t put_seedling4_path[PUT_SEEDLING4_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING3 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR-110.0f , CAR_Y_PUT_SEEDLING3 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR  -20.0f                        ,90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING4 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR        , CAR_Y_PUT_SEEDLING4 + Y_PUT_SEEDING1_MAP_ERROR - CAR_Y_MOVE_LOW_DISTANCE  - ACTION_ERROR     ,90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING4 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR-110.0f , CAR_Y_PUT_SEEDLING4 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR  - 5.0f                        ,90.0f, MAX_PLAN_VEL},
};

pose_t get_seedling3_path[GET_SEEDLING3_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING4 + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR-110.0f, CAR_Y_PUT_SEEDLING4 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR-5.0f , 90.0f, MAX_PLAN_VEL},
//	{CAR_X_GET_SEEDLING3                           , CAR_Y_PUT_SEEDLING4 + Y_PUT_SEEDING1_MAP_ERROR - 2*ACTION_ERROR,-180.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING3 + ACTION_ERROR  - 110.0f                        , CAR_Y_GET_SEEDLING3 - ACTION_ERROR - 20.0f                         , 90.0f, MAX_PLAN_VEL},
};

pose_t get_seedling4_path[GET_SEEDLING4_PATH_LEN]=
{
	{CAR_X_GET_SEEDLING3 + ACTION_ERROR-90.0f,               CAR_Y_GET_SEEDLING3 - ACTION_ERROR  -20.0f                 , 90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING3 + ACTION_ERROR,                     CAR_Y_GET_SEEDLING3 + CAR_Y_MOVE_LOW_DISTANCE- ACTION_ERROR,	90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING4 - ACTION_ERROR,                     CAR_Y_GET_SEEDLING3 + CAR_Y_MOVE_LOW_DISTANCE- ACTION_ERROR, -90.0f,  MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING4 - ACTION_ERROR-36.0f,               CAR_Y_GET_SEEDLING4 - ACTION_ERROR  -25.0f                 , -90.0f,  MAX_PLAN_VEL},
};

pose_t put_seedling5_path[PUT_SEEDLING5_PATH_LEN]=
{
	{CAR_X_GET_SEEDLING4 - ACTION_ERROR - SERVO_RIGHT_ERROR -36.0f      , CAR_Y_GET_SEEDLING4 - ACTION_ERROR  -25.0f         , -90.0f,  MAX_PLAN_VEL},
//	{CAR_X_GET_SEEDLING4 + ACTION_ERROR,CAR_Y_PUT_SEEDLING5 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR                        , -90.0f  , MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING5 - ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR -30.0f ,CAR_Y_PUT_SEEDLING5  - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR -28.0f  + 10.0f         , -90.0f  , MAX_PLAN_VEL},
};

pose_t put_seedling6_path[PUT_SEEDLING6_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING5  - ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR-30.0f,CAR_Y_PUT_SEEDLING5 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR    -28.0f  + 10.0f                        ,  -90.0f  , MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING6  - ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR,CAR_Y_PUT_SEEDLING6 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR - CAR_Y_MOVE_LOW_DISTANCE,  -90.0f  , MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING6  - ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR-30.0f,CAR_Y_PUT_SEEDLING6 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR -28.0f                         ,  -90.0f    , MAX_PLAN_VEL},
};

pose_t put_seedling7_path[PUT_SEEDLING7_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING6 - ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR -30.0f , CAR_Y_PUT_SEEDLING6 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR  -28.0f                      ,  -90.0f    , MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING7 + ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR, CAR_Y_PUT_SEEDLING7  - ACTION_ERROR+ Y_PUT_SEEDING1_MAP_ERROR - CAR_Y_MOVE_LOW_DISTANCE,  90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING7 + ACTION_ERROR+  X_PUT_SEEDING1_MAP_ERROR -80.0f ,  CAR_Y_PUT_SEEDLING7  - ACTION_ERROR+ Y_PUT_SEEDING1_MAP_ERROR-20.0f                          ,  90.0f, MAX_PLAN_VEL},
//	{CAR_X_PUT_SEEDLING7 +  X_PUT_SEEDING1_MAP_ERROR                         , CAR_Y_PUT_SEEDLING7+ Y_PUT_SEEDING1_MAP_ERROR -2*ACTION_ERROR, -180.0f, MAX_PLAN_VEL}
};

pose_t put_seedling8_path[PUT_SEEDLING8_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING7 + ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR -80.0f , CAR_Y_PUT_SEEDLING7 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR -20.0f                          , 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING8 + ACTION_ERROR+  X_PUT_SEEDING1_MAP_ERROR, CAR_Y_PUT_SEEDLING7 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR - CAR_Y_MOVE_LOW_DISTANCE  , 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING8 + ACTION_ERROR+  X_PUT_SEEDING1_MAP_ERROR -80.0f , CAR_Y_PUT_SEEDLING8 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR -20.0f                           , 90.0f, MAX_PLAN_VEL},
};

//纠偏
pose_t goto_zone2[GOTO_ZONE2_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING8 + ACTION_ERROR+  X_PUT_SEEDING1_MAP_ERROR-80.0f, CAR_Y_PUT_SEEDLING8 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR-20.0f                            , 90.0f, MAX_PLAN_VEL},
	{CAR_X_BEFORE_SLOPE, CAR_Y_BEFORE_SLOPE-40.0f,               180.0f, MAX_PLAN_VEL},
	{CAR_X_BEFORE_SLOPE, CAR_Y_DT35_CORRECT + SLOPE_DELTA, 0.0f, MAX_PLAN_VEL},
	{CAR_X_DT35_CORRECT, CAR_Y_DT35_CORRECT + SLOPE_DELTA, 0.0f, MAX_PLAN_VEL},
};


//二区取球
pose_t get_ball1[GET_BALL1_PATH_LEN]= 
{
	{CAR_X_GET_BALL1  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL1 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL1  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL1+20.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball2[GET_BALL2_PATH_LEN]= 
{
	{CAR_X_GET_BALL2  + X_ZONE2_MAP_ERROR-80.0f   , CAR_Y_GET_BALL2 + CAR_Y_MOVE_LOW_DISTANCE+20.0f, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL2  + X_ZONE2_MAP_ERROR-110.0f  , CAR_Y_GET_BALL2+50.0f, 													 0.0f, MAX_PLAN_VEL},
};

pose_t get_ball3[GET_BALL3_PATH_LEN]= 
{
	{CAR_X_GET_BALL3  + X_ZONE2_MAP_ERROR-110.0f, CAR_Y_GET_BALL3 + CAR_Y_MOVE_LOW_DISTANCE+50.0f, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL3  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL3+50.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball4[GET_BALL4_PATH_LEN]= 
{
	{CAR_X_GET_BALL4  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL4 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL4  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL4+50.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball5[GET_BALL5_PATH_LEN]= 
{
	{CAR_X_GET_BALL5  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL5 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL5  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL5+50.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball6[GET_BALL6_PATH_LEN]= 
{
	{CAR_X_GET_BALL6  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL6 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL6  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL6+50.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball7[GET_BALL7_PATH_LEN]= 
{
	{CAR_X_GET_BALL7  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL7 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL7  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL7+50.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball8[GET_BALL8_PATH_LEN]= 
{
	{CAR_X_GET_BALL8  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL8 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL8  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL8+50.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball9[GET_BALL9_PATH_LEN]= 
{
	{CAR_X_GET_BALL9  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL9 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL9  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL9+50.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball10[GET_BALL10_PATH_LEN]= 
{
	{CAR_X_GET_BALL10  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL10 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL10  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL10+50.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball11[GET_BALL11_PATH_LEN]= 
{
	{CAR_X_GET_BALL11  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL11 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL11  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL11+50.0f, 													0.0f, MAX_PLAN_VEL},
};
pose_t get_ball12[GET_BALL12_PATH_LEN]= 
{
	{CAR_X_GET_BALL12  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL12 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL12  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_GET_BALL12+50.0f, 													0.0f, MAX_PLAN_VEL},
};

//二区射球
pose_t shoot_ball1[SHOOT_BALL1_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL1  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL1+50.0f, CAR_YAW_SHOOT1,MAX_PLAN_VEL},
};

pose_t shoot_ball2[SHOOT_BALL2_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL2  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL2+50.0f, CAR_YAW_SHOOT2,MAX_PLAN_VEL},
};

pose_t shoot_ball3[SHOOT_BALL3_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL3  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL3+50.0f, CAR_YAW_SHOOT3,MAX_PLAN_VEL},
};
pose_t shoot_ball4[SHOOT_BALL4_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL4  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL4+50.0f, CAR_YAW_SHOOT4,MAX_PLAN_VEL},
};

pose_t shoot_ball5[SHOOT_BALL5_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL5  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL5+50.0f, CAR_YAW_SHOOT5,MAX_PLAN_VEL},
};

pose_t shoot_ball6[SHOOT_BALL6_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL6  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL6+50.0f, CAR_YAW_SHOOT6,MAX_PLAN_VEL},
};

pose_t shoot_ball7[SHOOT_BALL7_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL7  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL7,  0.0f          ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL7  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL7+50.0f,  CAR_YAW_SHOOT7,MAX_PLAN_VEL},
};

pose_t shoot_ball8[SHOOT_BALL8_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL8  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL8,  0.0f          ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL8  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL8+50.0f,  CAR_YAW_SHOOT8,MAX_PLAN_VEL},
};

pose_t shoot_ball9[SHOOT_BALL9_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL9  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL9,  0.0f          ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL9  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL9+50.0f,  CAR_YAW_SHOOT9,MAX_PLAN_VEL},
};

pose_t shoot_ball10[SHOOT_BALL10_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL10  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL10, 0.0f           ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL10  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL10+50.0f, CAR_YAW_SHOOT10,MAX_PLAN_VEL},
};

pose_t shoot_ball11[SHOOT_BALL11_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL11  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL11, 0.0f           ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL11  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL11+50.0f, CAR_YAW_SHOOT11,MAX_PLAN_VEL},
};

pose_t shoot_ball12[SHOOT_BALL12_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL12  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL12, 0.0f           ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL12  + X_ZONE2_MAP_ERROR-80.0f, CAR_Y_SHOOT_BALL12+50.0f, CAR_YAW_SHOOT12,MAX_PLAN_VEL},
};

/************************************红半场****************************************/
pose_t get_seedling1_path_red[GET_SEEDLING1_PATH_LEN]=
{
	{CAR_X_START                                                                 ,CAR_Y_START                                             ,0.0f ,   0.0f},
	{CAR_X_START                                                                 ,CAR_Y_START + CAR_Y_MOVE_LOW_DISTANCE                   ,0.0f ,   MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING1_RED - ACTION_ERROR                                      ,CAR_Y_START + CAR_Y_MOVE_LOW_DISTANCE        ,-90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING1_RED - ACTION_ERROR+85.0f                                ,CAR_Y_GET_SEEDLING1 - ACTION_ERROR + 50.0f   ,-90.0f,  MAX_PLAN_VEL},
};

pose_t get_seedling2_path_red[GET_SEEDLING2_PATH_LEN]=
{
	{CAR_X_GET_SEEDLING1_RED - ACTION_ERROR+85.0f             , CAR_Y_GET_SEEDLING1 - ACTION_ERROR+50.0f             ,-90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING1_RED - ACTION_ERROR                   , CAR_Y_GET_SEEDLING1 + CAR_Y_MOVE_LOW_DISTANCE- ACTION_ERROR ,-90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING2_RED + ACTION_ERROR                   , CAR_Y_GET_SEEDLING1 + CAR_Y_MOVE_LOW_DISTANCE- ACTION_ERROR ,90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING2_RED + ACTION_ERROR +35.0f            , CAR_Y_GET_SEEDLING2 - ACTION_ERROR+35.0f             ,90.0f, MAX_PLAN_VEL},
};

pose_t put_seedling1_path_red[PUT_SEEDLING1_PATH_LEN]=
{
	{CAR_X_GET_SEEDLING2_RED + ACTION_ERROR +35.0f            , CAR_Y_GET_SEEDLING2 - ACTION_ERROR+35.0f             ,90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING2_RED + ACTION_ERROR                                  , CAR_Y_PUT_SEEDLING1 - ACTION_ERROR                                                , 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING1_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR  +20.0f     , CAR_Y_PUT_SEEDLING1 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR + 40.0f   , 90.0f, MAX_PLAN_VEL},
};

pose_t put_seedling2_path_red[PUT_SEEDLING2_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING1_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR  +20.0f     , CAR_Y_PUT_SEEDLING1 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR + 40.0f, 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING2_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR , CAR_Y_PUT_SEEDLING1 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR + CAR_Y_MOVE_SMALL_DISTANCE, 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING2_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR , CAR_Y_PUT_SEEDLING2 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR +40.0f                     , 90.0f, MAX_PLAN_VEL},
};                                                       

pose_t put_seedling3_path_red[PUT_SEEDLING3_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING2_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR , CAR_Y_PUT_SEEDLING2 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR +40.0f                     , 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING3_RED  + ACTION_ERROR+ X_PUT_SEEDING1_MAP_ERROR, CAR_Y_PUT_SEEDLING2 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR  +140.0f       ,90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING3_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR+20.0f, CAR_Y_PUT_SEEDLING3 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR +60.0f   ,90.0f, MAX_PLAN_VEL},
//	{CAR_X_PUT_SEEDLING3                                      + X_PUT_SEEDING1_MAP_ERROR , CAR_Y_PUT_SEEDLING3 + Y_PUT_SEEDING1_MAP_ERROR - 2*ACTION_ERROR + SERVO_RIGHT_ERROR,           -180.0f, MAX_PLAN_VEL},
};                                                          

pose_t put_seedling4_path_red[PUT_SEEDLING4_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING3_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR+20.0f, CAR_Y_PUT_SEEDLING3 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR +60.0f   ,90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING3_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR, CAR_Y_PUT_SEEDLING4 + Y_PUT_SEEDING1_MAP_ERROR - CAR_Y_MOVE_LOW_DISTANCE  - ACTION_ERROR      ,90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING4_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR  , CAR_Y_PUT_SEEDLING4 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR +60.0f                          ,90.0f, MAX_PLAN_VEL},
};

pose_t get_seedling3_path_red[GET_SEEDLING3_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING4_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR  , CAR_Y_PUT_SEEDLING4 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR +60.0f                          ,90.0f, MAX_PLAN_VEL},
//	{CAR_X_GET_SEEDLING3                           , CAR_Y_PUT_SEEDLING4 + Y_PUT_SEEDING1_MAP_ERROR - 2*ACTION_ERROR,-180.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING3_RED + ACTION_ERROR + 30.0f                    , CAR_Y_GET_SEEDLING3 - ACTION_ERROR + 65.0f                                                                     ,90.0f, MAX_PLAN_VEL},
};

pose_t get_seedling4_path_red[GET_SEEDLING4_PATH_LEN]=
{
	{CAR_X_GET_SEEDLING3_RED + ACTION_ERROR + 30.0f, CAR_Y_GET_SEEDLING3 - ACTION_ERROR+65.0f                                                                     ,90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING3_RED + ACTION_ERROR,                     CAR_Y_GET_SEEDLING3 + CAR_Y_MOVE_LOW_DISTANCE- ACTION_ERROR,	90.0f, MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING4_RED - ACTION_ERROR        , CAR_Y_GET_SEEDLING3 + CAR_Y_MOVE_LOW_DISTANCE- ACTION_ERROR, -90.0f,  MAX_PLAN_VEL},
	{CAR_X_GET_SEEDLING4_RED - ACTION_ERROR +75.0f, CAR_Y_GET_SEEDLING4 - ACTION_ERROR  +70.0f                       , -90.0f,  MAX_PLAN_VEL},
};

pose_t put_seedling5_path_red[PUT_SEEDLING5_PATH_LEN]=
{
	{CAR_X_GET_SEEDLING4_RED - ACTION_ERROR +75.0f, CAR_Y_GET_SEEDLING4 - ACTION_ERROR  +70.0f                       , -90.0f,  MAX_PLAN_VEL},
//	{CAR_X_GET_SEEDLING4 + ACTION_ERROR,CAR_Y_PUT_SEEDLING5 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR                        , -90.0f  , MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING5_RED - ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR + 80.0f,CAR_Y_PUT_SEEDLING5 + Y_PUT_SEEDING1_MAP_ERROR- ACTION_ERROR  +40.0f                         ,  -90.0f  , MAX_PLAN_VEL},
};

pose_t put_seedling6_path_red[PUT_SEEDLING6_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING5_RED - ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR  + 80.0f,CAR_Y_PUT_SEEDLING5 + Y_PUT_SEEDING1_MAP_ERROR- ACTION_ERROR +40.0f                          ,  -90.0f  , MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING6_RED - ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR,CAR_Y_PUT_SEEDLING6 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR- CAR_Y_MOVE_LOW_DISTANCE,  -90.0f  , MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING6_RED - ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR + 80.0f,CAR_Y_PUT_SEEDLING6 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR   +40.0f                      ,  -90.0f    , MAX_PLAN_VEL},
};

pose_t put_seedling7_path_red[PUT_SEEDLING7_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING6_RED - ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR + 80.0f ,CAR_Y_PUT_SEEDLING6 + Y_PUT_SEEDING1_MAP_ERROR - ACTION_ERROR +40.0f                        ,  -90.0f    , MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING7_RED + ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR                         , CAR_Y_PUT_SEEDLING7 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR - CAR_Y_MOVE_LOW_DISTANCE,  90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING7_RED + ACTION_ERROR +  X_PUT_SEEDING1_MAP_ERROR +20.0f       , CAR_Y_PUT_SEEDLING7 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR+40.0f                          ,  90.0f, MAX_PLAN_VEL},
//	{CAR_X_PUT_SEEDLING7 +  X_PUT_SEEDING1_MAP_ERROR                         , CAR_Y_PUT_SEEDLING7+ Y_PUT_SEEDING1_MAP_ERROR -2*ACTION_ERROR, -180.0f, MAX_PLAN_VEL}
};

pose_t put_seedling8_path_red[PUT_SEEDLING8_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING7_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR +20.0f, CAR_Y_PUT_SEEDLING7 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR   +40.0f                   , 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING8_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR +20.0f, CAR_Y_PUT_SEEDLING7 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR - CAR_Y_MOVE_LOW_DISTANCE  , 90.0f, MAX_PLAN_VEL},
	{CAR_X_PUT_SEEDLING8_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR +20.0f, CAR_Y_PUT_SEEDLING8 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR                    +40.0f  , 90.0f, MAX_PLAN_VEL}
};


//纠偏
pose_t goto_zone2_red[GOTO_ZONE2_PATH_LEN]=
{
	{CAR_X_PUT_SEEDLING8_RED + ACTION_ERROR + X_PUT_SEEDING1_MAP_ERROR, CAR_Y_PUT_SEEDLING8 - ACTION_ERROR + Y_PUT_SEEDING1_MAP_ERROR                    +40.0f  , 90.0f, MAX_PLAN_VEL},
	{CAR_X_BEFORE_SLOPE_RED, CAR_Y_BEFORE_SLOPE,               90.0f, MAX_PLAN_VEL},
	{CAR_X_BEFORE_SLOPE_RED, CAR_Y_BEFORE_SLOPE + SLOPE_DELTA,               90.0f, MAX_PLAN_VEL},
//	{CAR_X_BEFORE_SLOPE_RED, CAR_Y_DT35_CORRECT + SLOPE_DELTA, 0.0f, MAX_PLAN_VEL},
	{CAR_X_DT35_CORRECT_RED, CAR_Y_DT35_CORRECT + SLOPE_DELTA, 0.0f, MAX_PLAN_VEL},};


//二区取球
pose_t get_ball1_red[GET_BALL1_PATH_LEN]= 
{
	{CAR_X_GET_BALL1_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL1 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL1_RED  + X_ZONE2_MAP_ERROR- 90.0f, CAR_Y_GET_BALL1+90.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball2_red[GET_BALL2_PATH_LEN]= 
{
	{CAR_X_GET_BALL2_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL2 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL2_RED  + X_ZONE2_MAP_ERROR- 120.0f, CAR_Y_GET_BALL2 +60.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball3_red[GET_BALL3_PATH_LEN]= 
{
	{CAR_X_GET_BALL3_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL3 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL3_RED  + X_ZONE2_MAP_ERROR- 90.0f, CAR_Y_GET_BALL3 +90.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball4_red[GET_BALL4_PATH_LEN]= 
{
	{CAR_X_GET_BALL4_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL4 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL4_RED  + X_ZONE2_MAP_ERROR- 90.0f, CAR_Y_GET_BALL4 +90.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball5_red[GET_BALL5_PATH_LEN]= 
{
	{CAR_X_GET_BALL5_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL5 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL5_RED  + X_ZONE2_MAP_ERROR- 100.0f, CAR_Y_GET_BALL5  +78.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball6_red[GET_BALL6_PATH_LEN]= 
{
	{CAR_X_GET_BALL6_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL6 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL6_RED  + X_ZONE2_MAP_ERROR - 90.0f, CAR_Y_GET_BALL6  +70.0f, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball7_red[GET_BALL7_PATH_LEN]= 
{
	{CAR_X_GET_BALL7_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL7 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL7_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL7, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball8_red[GET_BALL8_PATH_LEN]= 
{
	{CAR_X_GET_BALL8_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL8 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL8_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL8, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball9_red[GET_BALL9_PATH_LEN]= 
{
	{CAR_X_GET_BALL9_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL9 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL9_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL9, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball10_red[GET_BALL10_PATH_LEN]= 
{
	{CAR_X_GET_BALL10_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL10 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL10_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL10, 													0.0f, MAX_PLAN_VEL},
};

pose_t get_ball11_red[GET_BALL11_PATH_LEN]= 
{
	{CAR_X_GET_BALL11_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL11 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL11_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL11, 													0.0f, MAX_PLAN_VEL},
};
pose_t get_ball12_red[GET_BALL12_PATH_LEN]= 
{
	{CAR_X_GET_BALL12_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL12 + CAR_Y_MOVE_LOW_DISTANCE, 0.0f, MAX_PLAN_VEL},
	{CAR_X_GET_BALL12_RED  + X_ZONE2_MAP_ERROR, CAR_Y_GET_BALL12, 													0.0f, MAX_PLAN_VEL},
};

//二区射球
pose_t shoot_ball1_red[SHOOT_BALL1_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL1_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL1, CAR_YAW_SHOOT1,MAX_PLAN_VEL},
};

pose_t shoot_ball2_red[SHOOT_BALL2_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL2_RED  + X_ZONE2_MAP_ERROR- 120.0f, CAR_Y_SHOOT_BALL2+60.0f, CAR_YAW_SHOOT2,MAX_PLAN_VEL},
};

pose_t shoot_ball3_red[SHOOT_BALL3_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL3_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL3, CAR_YAW_SHOOT3,MAX_PLAN_VEL},
};
pose_t shoot_ball4_red[SHOOT_BALL4_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL4_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL4, CAR_YAW_SHOOT4,MAX_PLAN_VEL},
};

pose_t shoot_ball5_red[SHOOT_BALL5_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL5_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL5, CAR_YAW_SHOOT5,MAX_PLAN_VEL},
};

pose_t shoot_ball6_red[SHOOT_BALL6_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL6_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL6, CAR_YAW_SHOOT6,MAX_PLAN_VEL},
};

pose_t shoot_ball7_red[SHOOT_BALL7_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL7_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL7,  0.0f          ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL7_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL7,  CAR_YAW_SHOOT7,MAX_PLAN_VEL},
};

pose_t shoot_ball8_red[SHOOT_BALL8_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL8_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL8,  0.0f          ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL8_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL8,  CAR_YAW_SHOOT8,MAX_PLAN_VEL},
};

pose_t shoot_ball9_red[SHOOT_BALL9_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL9_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL9,  0.0f          ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL9_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL9,  CAR_YAW_SHOOT9,MAX_PLAN_VEL},
};

pose_t shoot_ball10_red[SHOOT_BALL10_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL10_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL10, 0.0f           ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL10_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL10, CAR_YAW_SHOOT10,MAX_PLAN_VEL},
};

pose_t shoot_ball11_red[SHOOT_BALL11_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL11_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL11, 0.0f           ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL11_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL11, CAR_YAW_SHOOT11,MAX_PLAN_VEL},
};

pose_t shoot_ball12_red[SHOOT_BALL12_PATH_LEN]= 
{
	{CAR_X_SHOOT_BALL12_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL12, 0.0f           ,MAX_PLAN_VEL},
	{CAR_X_SHOOT_BALL12_RED  + X_ZONE2_MAP_ERROR, CAR_Y_SHOOT_BALL12, CAR_YAW_SHOOT12,MAX_PLAN_VEL},
};


/**
  * @brief    路径跟随
  * @retval   1 跑到点   0 没跑到
  */
uint8_t path_flow_positon(chassis_move_t *chassis_flow, pose_t *path, uint8_t path_len)
{
	  uint8_t target = chassis_flow->target;
	
		float target_dis2FinalX = path[path_len - 1].position.x - chassis_flow->ops.pos_x; // 终点，x轴的距离 = 当前位置点x坐标 - 目标点x坐标
		float target_dis2FinalY = path[path_len - 1].position.y - chassis_flow->ops.pos_y; // 终点，y轴的距离 = 当前位置点y坐标 - 目标点y坐标
		float target_yaw2Final  = path[path_len - 1].direction - chassis_flow->ops.car_total_yaw ;//终点，车体航向角（世界坐标系）
		
		float dis2FinalX = path[target].position.x - chassis_flow->ops.pos_x; // 要走到目标点，x轴的距离 = 当前位置点x坐标 - 目标点x坐标
		float dis2FinalY = path[target].position.y - chassis_flow->ops.pos_y; // 要走到目标点，y轴的距离 = 当前位置点y坐标 - 目标点y坐标
		float yaw2Final = path[target].direction - chassis_flow->ops.car_total_yaw ;//要走到目标点，车体航向角（世界坐标系）
		
		if(ABS(target_dis2FinalX) < PATH_X_DEAD && ABS(target_dis2FinalY) < PATH_Y_DEAD && target==path_len-1 && ABS(target_yaw2Final) < PATH_W_DEAD && ave_speed < PATH_SPEED_DEAD)  
		{
			chassis_flow->vx_set=0;
			chassis_flow->vy_set=0;
			chassis_flow->wz_set=0;
			return 1;
		}  		
//	static uint8_t flag = 0;
  chassis_flow->car_yaw_set= path[target].direction;//转向
	chassis_flow->pos_x_target  = path[target].position.x;
	chassis_flow->pos_y_target  = path[target].position.y;
	

		
		if(ABS(dis2FinalX) < PATH_X_DEAD && ABS(dis2FinalY) < PATH_Y_DEAD && target!=path_len && ABS(yaw2Final) < PATH_W_DEAD)
		{
			target=target+1;
			if(target > path_len-1)
			{
				target = path_len-1;
			}
			chassis_flow->target =target;
		}
		
		return 0;
}	

///**
//  * @brief    直接由当前位置跑到目标点
//  * @retval   1 跑到点   0 没跑到
//	* @note     本来用来跑二区的点,现在不用了
//  */
//uint8_t point_flow_positon(chassis_move_t *chassis_flow, pose_t target_point)
//{
//	pose_t temp_path[2]=
//	{
//		{chassis_flow->ops.pos_x,chassis_flow->ops.pos_y, chassis_flow->ops.car_total_yaw,MAX_PLAN_VEL},
//		{target_point.position.x,target_point.position.y, target_point.direction,         MAX_PLAN_VEL},
//	};
//	
//	uint8_t status = path_flow_positon(chassis_flow,temp_path,2);
//	return status;
//}

///**
//  * @brief    匀速路径跟随
//  * @param    chassis_flow 底盘
//	* @param    path 路径
//	* @param    path_len 路径中点的个数
//  * @retval   0没到终点，1到终点
//  */
//uint8_t path_flow_speed(chassis_move_t *chassis_flow,pose_t *path,uint8_t path_len)
//{
//  static uint8_t target = 0;
//	if(target==path_len)
//	{
//		//跑到目标点，开始纠偏
//		chassis_flow->car_yaw_set= path[target-1].direction;
//		chassis_flow->pos_x_target  = path[target-1].position.x;
//		chassis_flow->pos_y_target  = path[target-1].position.y;
//		
//		return 1;
//	}
//	
//  float dis2FinalX = path[target].position.x - chassis_flow->ops.pos_x; // 要走到目标点，x轴的距离 = 当前位置点x坐标 - 目标点x坐标
//  float dis2FinalY = path[target].position.y - chassis_flow->ops.pos_y; // 要走到目标点，y轴的距离 = 当前位置点y坐标 - 目标点y坐标
//  float yaw2Final = path[target].direction - chassis_flow->ops.car_total_yaw ;//要走到目标点，车体航向角（世界坐标系）
//	float dis=_sqrtf(dis2FinalX*dis2FinalX+dis2FinalY*dis2FinalY);
//	
//	chassis_flow->car_vel_set=path[target].vel;
//	chassis_flow->car_dis_set=R2DEG(atan2(dis2FinalX,dis2FinalY));
//	chassis_flow->car_yaw_set=path[target].direction;
//	
//	if(dis< 50.0f && ABS(yaw2Final) < 2.0f)
//	{
//		target=target+1;
//		chassis_flow->target =target;
//	}
//	return 0;
//}
/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/

