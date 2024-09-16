/**
  ******************************************************************************
  * @file     chassis_task.c/h
  * @author   Epoch Robotics 2024
  * @version  �汾��
  * @date     03-28-2024
  * @brief    ȫ���ֵ��̿������� ���Զ�����
  ******************************************************************************
  * @attention
  * ע������
  * 
  * 
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/

#include "chassis_task.h"
#include "math.h"
#include "path.h"
#include "config.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
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
		
		
//#define abs_minus(input, output, minus)              \
//{																									 	\ 
//		if ((input) > (dealine) || (input) < -(dealine)) \
//		{                                                \
//				(output) = (input);                          \
//		}                                                \
//		else                                             \
//		{                                                \
//				(output) = 0;                                \
//		}                                                \
//}
/* Private  variables ---------------------------------------------------------*/
//�����˶�����
chassis_move_t chassis_move;
//�ĸ�����ƽ��ת��
int16_t ave_speed;
//���̱�־λ��������
uint32_t chassis_test1=0;
uint32_t chassis_test2=0;
uint32_t chassis_test3=0;
uint32_t chassis_test4=0;
uint32_t chassis_test5=0;
uint32_t chassis_test6=0;
uint32_t chassis_test7=0;
uint32_t chassis_test8=0;
uint32_t chassis_test9=0;
uint32_t chassis_test10=0;
uint32_t chassis_test11=0;	
uint32_t chassis_test12=0;	
int8_t   chassis_test13=0;	
float test_x = 0;
float test_y = 0;		
/* Extern   variables ---------------------------------------------------------*/
extern motor_measure_t motor_chassis[4];
extern osMessageQId ChassisToRobotHandle;
extern osMessageQId RobotToChassisHandle;
extern osSemaphoreId RobotChassisSemphoreHandle;

extern pose_t get_seedling1_path[GET_SEEDLING1_PATH_LEN];
extern pose_t get_seedling2_path[GET_SEEDLING2_PATH_LEN];
extern pose_t put_seedling1_path[PUT_SEEDLING1_PATH_LEN];
extern pose_t put_seedling2_path[PUT_SEEDLING2_PATH_LEN];
extern pose_t put_seedling3_path[PUT_SEEDLING3_PATH_LEN];
extern pose_t put_seedling4_path[PUT_SEEDLING4_PATH_LEN];
extern pose_t get_seedling3_path[GET_SEEDLING3_PATH_LEN];
extern pose_t get_seedling4_path[GET_SEEDLING4_PATH_LEN];
extern pose_t put_seedling5_path[PUT_SEEDLING5_PATH_LEN];
extern pose_t put_seedling6_path[PUT_SEEDLING6_PATH_LEN];
extern pose_t put_seedling7_path[PUT_SEEDLING7_PATH_LEN];
extern pose_t put_seedling8_path[PUT_SEEDLING8_PATH_LEN];

extern pose_t goto_zone2[GOTO_ZONE2_PATH_LEN];

extern pose_t get_ball1[GET_BALL1_PATH_LEN];
extern pose_t get_ball2[GET_BALL2_PATH_LEN];
extern pose_t get_ball3[GET_BALL3_PATH_LEN];
extern pose_t get_ball4[GET_BALL4_PATH_LEN];
extern pose_t get_ball5[GET_BALL5_PATH_LEN];
extern pose_t get_ball6[GET_BALL6_PATH_LEN];
extern pose_t get_ball7[GET_BALL7_PATH_LEN];
extern pose_t get_ball8[GET_BALL8_PATH_LEN];
extern pose_t get_ball9[GET_BALL9_PATH_LEN];
extern pose_t get_ball10[GET_BALL10_PATH_LEN];
extern pose_t get_ball11[GET_BALL11_PATH_LEN];
extern pose_t get_ball12[GET_BALL12_PATH_LEN];

extern pose_t shoot_ball1[SHOOT_BALL1_PATH_LEN];
extern pose_t shoot_ball2[SHOOT_BALL2_PATH_LEN];
extern pose_t shoot_ball3[SHOOT_BALL3_PATH_LEN];
extern pose_t shoot_ball4[SHOOT_BALL3_PATH_LEN];
extern pose_t shoot_ball5[SHOOT_BALL5_PATH_LEN];
extern pose_t shoot_ball6[SHOOT_BALL6_PATH_LEN];
extern pose_t shoot_ball7[SHOOT_BALL7_PATH_LEN];
extern pose_t shoot_ball8[SHOOT_BALL8_PATH_LEN];
extern pose_t shoot_ball9[SHOOT_BALL9_PATH_LEN];
extern pose_t shoot_ball10[SHOOT_BALL10_PATH_LEN];
extern pose_t shoot_ball11[SHOOT_BALL11_PATH_LEN];
extern pose_t shoot_ball12[SHOOT_BALL12_PATH_LEN];

extern pose_t get_seedling1_path_red[GET_SEEDLING1_PATH_LEN];
extern pose_t get_seedling2_path_red[GET_SEEDLING2_PATH_LEN];
extern pose_t put_seedling1_path_red[PUT_SEEDLING1_PATH_LEN];
extern pose_t put_seedling2_path_red[PUT_SEEDLING2_PATH_LEN];
extern pose_t put_seedling3_path_red[PUT_SEEDLING3_PATH_LEN];
extern pose_t put_seedling4_path_red[PUT_SEEDLING4_PATH_LEN];
extern pose_t get_seedling3_path_red[GET_SEEDLING3_PATH_LEN];
extern pose_t get_seedling4_path_red[GET_SEEDLING4_PATH_LEN];
extern pose_t put_seedling5_path_red[PUT_SEEDLING5_PATH_LEN];
extern pose_t put_seedling6_path_red[PUT_SEEDLING6_PATH_LEN];
extern pose_t put_seedling7_path_red[PUT_SEEDLING7_PATH_LEN];
extern pose_t put_seedling8_path_red[PUT_SEEDLING8_PATH_LEN];

extern pose_t goto_zone2_red[GOTO_ZONE2_PATH_LEN];

extern pose_t get_ball1_red[GET_BALL1_PATH_LEN];
extern pose_t get_ball2_red[GET_BALL2_PATH_LEN];
extern pose_t get_ball3_red[GET_BALL3_PATH_LEN];
extern pose_t get_ball4_red[GET_BALL4_PATH_LEN];
extern pose_t get_ball5_red[GET_BALL5_PATH_LEN];
extern pose_t get_ball6_red[GET_BALL6_PATH_LEN];
extern pose_t get_ball7_red[GET_BALL7_PATH_LEN];
extern pose_t get_ball8_red[GET_BALL8_PATH_LEN];
extern pose_t get_ball9_red[GET_BALL9_PATH_LEN];
extern pose_t get_ball10_red[GET_BALL10_PATH_LEN];
extern pose_t get_ball11_red[GET_BALL11_PATH_LEN];
extern pose_t get_ball12_red[GET_BALL12_PATH_LEN];

extern pose_t shoot_ball1_red[SHOOT_BALL1_PATH_LEN];
extern pose_t shoot_ball2_red[SHOOT_BALL2_PATH_LEN];
extern pose_t shoot_ball3_red[SHOOT_BALL3_PATH_LEN];
extern pose_t shoot_ball4_red[SHOOT_BALL3_PATH_LEN];
extern pose_t shoot_ball5_red[SHOOT_BALL5_PATH_LEN];
extern pose_t shoot_ball6_red[SHOOT_BALL6_PATH_LEN];
extern pose_t shoot_ball7_red[SHOOT_BALL7_PATH_LEN];
extern pose_t shoot_ball8_red[SHOOT_BALL8_PATH_LEN];
extern pose_t shoot_ball9_red[SHOOT_BALL9_PATH_LEN];
extern pose_t shoot_ball10_red[SHOOT_BALL10_PATH_LEN];
extern pose_t shoot_ball11_red[SHOOT_BALL11_PATH_LEN];
extern pose_t shoot_ball12_red[SHOOT_BALL12_PATH_LEN];

extern dt35_t dt35[4];
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_terminal(chassis_move_t *chassis_terminal);
static void chassis_manual_control(chassis_move_t *chassis_move_control);
static void chassis_auto_control(chassis_move_t *chassis_move_control);
static void chassis_cmd_error(chassis_move_t *chassis_control) ;		
static void chassis_control_loop(chassis_move_t *chassis_control);
/* Private  functions ---------------------------------------------------------*/
static void chassis_key_handle_zone1(chassis_move_t *chassis_key_set);
static void chassis_key_handle_zone2(chassis_move_t *chassis_key_set);

		
/**
* @brief          ��������
* @param[in]      argument: ��
* @retval         none
*/
void chassis_task(void const * argument)
{
  //����һ��ʱ��
  chassis_init(&chassis_move);
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	vTaskDelay(16000);
  while(1)
  {
		chassis_set_terminal(&chassis_move);
		switch(chassis_move.control_mode)
		{
			case CHASSIS_MANUAL:
				chassis_manual_control(&chassis_move);
				break; 
			case CHASSIS_AUTO:
				chassis_auto_control(&chassis_move);
				break;
			case CHASSIS_CMD_ERROR:
				chassis_cmd_error(&chassis_move);
				break;
		}
		chassis_control_loop(&chassis_move);
//		dt35_send(); 
//		vTaskDelay(50);
	  vTaskDelay(5);
//		dt35_send();
//		test_x = CAR_X_OFFSET - CHASSIS_LEN_HALF -dt35_read(&dt35[0]);
//		test_y = (MAP_Y_BOARD - dt35_read(&dt35[2]) - CAR_Y_OFFSET - CHASSIS_LEN_HALF);

//		vTaskDelay(50);
  }	
}

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ��,
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    chassis_move_init->chassis_RC = get_remote_control_point();
//		//��ȡң����ָ��
//    chassis_move_init->chassis_RC = get_remote_point();
	
    //��һ���˲�����б�º�������
    const static fp32 chassis_vel_order_filter[1] = {CHASSIS_ACCEL_VEL};
    const static fp32 chassis_yaw_order_filter[1] = {CHASSIS_ACCEL_YAW};

    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx  , CHASSIS_CONTROL_TIME, chassis_vel_order_filter);
		first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy  , CHASSIS_CONTROL_TIME, chassis_vel_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_wz  , CHASSIS_CONTROL_TIME, chassis_yaw_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_zero, CHASSIS_CONTROL_TIME, chassis_yaw_order_filter);
	  //pid��ʼ��
		pid_init(&chassis_move_init->motor_pid[0]);
		chassis_move_init->motor_pid[0].f_param_init(&chassis_move_init->motor_pid[0], PID_Speed, 
													   16000,5000,30,0,800,0,28,0.6,0,PID_IMPROVE_NONE);
 		pid_init(&chassis_move_init->motor_pid[1]);
		chassis_move_init->motor_pid[1].f_param_init(&chassis_move_init->motor_pid[1], PID_Speed, 
													   16000,5000,30,0,800,0,28,0.6,0,PID_IMPROVE_NONE);
		pid_init(&chassis_move_init->motor_pid[2]);
		chassis_move_init->motor_pid[2].f_param_init(&chassis_move_init->motor_pid[2], PID_Speed, 
													   16000,5000,30,0,800,0,28,0.6,0,PID_IMPROVE_NONE);
		pid_init(&chassis_move_init->motor_pid[3]);
		chassis_move_init->motor_pid[3].f_param_init(&chassis_move_init->motor_pid[3], PID_Speed, 
													   16000,5000,30,0,800,0,28,  0.6  ,0,PID_IMPROVE_NONE);
		
		pid_init(&chassis_move_init->yaw_correct_pid);
		chassis_move_init->yaw_correct_pid.f_param_init(&chassis_move_init->yaw_correct_pid, PID_Position, 3000, 5000, 
                        2, 0, 0, 0, 50, 0, 0, PID_IMPROVE_NONE);
		pid_init(&chassis_move_init->yaw_pid);
		chassis_move_init->yaw_pid.f_param_init(&chassis_move_init->yaw_pid, PID_Position, 3000, 5000, 
                        0.5, 0, 0, 0, 50, 0, 5, PID_IMPROVE_NONE);
		pid_init(&chassis_move_init->pos_x_pid);
		chassis_move_init->pos_x_pid.f_param_init(&chassis_move_init->pos_x_pid, PID_Position, 2000, 5000, 
                        5, 0, 0, 0, 20, 0.0, 5, PID_IMPROVE_NONE);
		pid_init(&chassis_move_init->pos_y_pid);
		chassis_move_init->pos_y_pid.f_param_init(&chassis_move_init->pos_y_pid, PID_Position, 2000, 5000, 
                        5, 0, 0, 0, 14, 0.0, 0, PID_IMPROVE_NONE);										

    //dt35��ʼ��
		usart_init();

		
		chassis_move_init->control_mode  = CHASSIS_MANUAL;
		chassis_move_init->manual_status = MANUAL;
		chassis_move_init->auto_status   = GET_SEEDLING1;
//		chassis_move_init->zone = CHASSIS_BLUE;


		chassis_move_init->zone          = ZONE_VALUE;
		chassis_move_init->game          = GAME_KIND;

////�쳡����
//		chassis_move_init->auto_status   = GOTO_ZONE2;

		//chassis_move_init->court = CHASSIS_BLUE; 
		 


		
#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
		
		//ֱ�Ӵ�2����ʼ����,Ҫ�ȸı�����
//		Update_X(CAR_X_ZONE2_TEST);
//		vTaskDelay(10);
//		Update_Y(CAR_Y_ZONE2_TEST);
//		vTaskDelay(10);
}

/**
 * @brief          �������Զ�,���ݵ���ʵʱ����ѡȡ���ʵ�У׼��
 * @param[in]      chassis_adjust:"chassis_move"����ָ��.
 * @retval         ѡ��������
 */
manual_e chassis_zone2_adjust(chassis_move_t *chassis_adjust)
{ 
	fp32 zone2_position_x = chassis_adjust->ops.pos_x;
	//���볡
	if(chassis_adjust->zone == CHASSIS_BLUE)
	{
		if((zone2_position_x > (CAR_X_GET_BALL1 + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL1 + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL1;
		}
		else if((zone2_position_x > (CAR_X_GET_BALL2 + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL2 + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL2;
		}
		else if((zone2_position_x > (CAR_X_GET_BALL3 + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL3 + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL3;
		}
		else if((zone2_position_x > (CAR_X_GET_BALL4 + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL4 + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL4;
		}
		else if((zone2_position_x > (CAR_X_GET_BALL5 + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL5 + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL5;
		}
		else if((zone2_position_x > (CAR_X_GET_BALL6 + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL6 + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL6;
		}
		else
		{
			return MANUAL_ERROR;
		}
	}
	//��볡
	else
	{
		if((zone2_position_x > (CAR_X_GET_BALL1_RED + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL1_RED + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL1_RED;
		}
		else if((zone2_position_x > (CAR_X_GET_BALL2_RED + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL2_RED + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL2_RED;
		}
		else if((zone2_position_x > (CAR_X_GET_BALL3_RED + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL3_RED + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL3_RED;
		}
		else if((zone2_position_x > (CAR_X_GET_BALL4_RED + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL4_RED + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL4_RED;
		}
		else if((zone2_position_x > (CAR_X_GET_BALL5_RED + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL5_RED + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL5_RED;
		}
		else if((zone2_position_x > (CAR_X_GET_BALL6_RED + X_ZONE2_MAP_ERROR - ZONE2_BALL_GAP_HALF)) && (zone2_position_x < (CAR_X_GET_BALL6_RED + X_ZONE2_MAP_ERROR + ZONE2_BALL_GAP_HALF)))
		{
			return CHASSIS_GET_BALL6_RED;
		}
		else
		{
			return MANUAL_ERROR;
		}		
	}
}

/**
  * @brief          ���̿���̨,���õ��̵Ŀ���״̬
  * @param[out]     chassis_move_control:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_terminal(chassis_move_t *chassis_terminal)
{	
	static uint32_t rx_buff;

	
	if(chassis_terminal->chassis_RC->sw[RC_SW_L_CHANNEL] == RC_SW_UP)
	{
		chassis_terminal->control_mode = CHASSIS_MANUAL;
		if(chassis_terminal->chassis_RC->sw[RC_SW_R_CHANNEL] == RC_SW_DOWN)
		{
			if(chassis_terminal->chassis_RC->mKey == KEY_R_U)
			{
				//chassis_terminal->manual_status = CHASSIS_ZONE2_UP;
				chassis_test13 = chassis_zone2_adjust(chassis_terminal);
				if(chassis_zone2_adjust(chassis_terminal) == MANUAL_ERROR)
				{
					chassis_terminal->manual_status = MANUAL;
					return ;
				}
				chassis_terminal->manual_status = chassis_zone2_adjust(chassis_terminal);
			}
			if(chassis_terminal->chassis_RC->mKey == KEY_R_D)
			{
				//chassis_terminal->manual_status = CHASSIS_ZONE2_DOWN;
				if(chassis_zone2_adjust(chassis_terminal) == MANUAL_ERROR)
				{
					chassis_terminal->manual_status = MANUAL;
					return ;
				}
				chassis_terminal->manual_status = (manual_e)(chassis_zone2_adjust(chassis_terminal) + CHASSIS_GET_BALL7 - CHASSIS_GET_BALL1);
			}
		}
		
		if(xQueueReceive(RobotToChassisHandle,&rx_buff,0) == pdTRUE)
		{
			chassis_terminal->manual_status = (manual_e)rx_buff;
		}
	}
	else if(chassis_terminal->chassis_RC->sw[RC_SW_L_CHANNEL] == RC_SW_DOWN )
	{
		chassis_terminal->control_mode = CHASSIS_AUTO;
		
		if(xQueueReceive(RobotToChassisHandle,&rx_buff,0) == pdTRUE)
		{
			chassis_terminal->auto_status = (auto_e)rx_buff;
		}
	}
	else
	{
		chassis_terminal->control_mode = CHASSIS_CMD_ERROR;
	}
	
		//�����볡״̬�л�
	if(chassis_terminal->zone == CHASSIS_RED)
	{
		if(chassis_terminal->manual_status >= CHASSIS_GET_BALL1 && chassis_terminal->manual_status <= CHASSIS_SHOOT_BALL12)
		{
			chassis_terminal->manual_status += (manual_e)(CHASSIS_GET_BALL1_RED - CHASSIS_GET_BALL1);
		}
		if(chassis_terminal->auto_status >= GET_SEEDLING1 && chassis_terminal->auto_status <= DT35_CORRECT)
		{
			chassis_terminal->auto_status += (auto_e)(GET_SEEDLING1_RED - GET_SEEDLING1);
		}
	}
	if(chassis_terminal->zone == CHASSIS_BLUE)
	{
		if(chassis_terminal->manual_status >= CHASSIS_GET_BALL1_RED && chassis_terminal->manual_status <= CHASSIS_SHOOT_BALL12_RED)
		{
			chassis_terminal->manual_status -= (manual_e)(CHASSIS_GET_BALL1_RED - CHASSIS_GET_BALL1);
		}
		if(chassis_terminal->auto_status >= GET_SEEDLING1_RED && chassis_terminal->auto_status <= DT35_CORRECT_RED)
		{
			chassis_terminal->auto_status -= (auto_e)(GET_SEEDLING1_RED - GET_SEEDLING1);
		}
	}

/****************************************************************************************************************************/
	
//	if(chassis_terminal->chassis_RC->remote.sw2 == SW_UP)
//	{
//		//��ҡ�����Զ�
//		chassis_terminal->control_mode = CHASSIS_MANUAL;
//		
//		//ͨ�����������жϽ�Ҫȡ��һ�Ż��ǵڶ��ŵ���
//		if(chassis_terminal->chassis_RC->remote.key6 == SINGLE_CLICK)
//		{
//			chassis_test13 = chassis_zone2_adjust(chassis_terminal);
//			if(chassis_zone2_adjust(chassis_terminal) == MANUAL_ERROR)
//			{
//				chassis_terminal->manual_status = MANUAL;
//				return ;
//			}
//			chassis_terminal->manual_status = chassis_zone2_adjust(chassis_terminal);
//		}
//		if(chassis_terminal->chassis_RC->remote.key5 == SINGLE_CLICK)
//		{
//			if(chassis_zone2_adjust(chassis_terminal) == MANUAL_ERROR)
//			{
//				chassis_terminal->manual_status = MANUAL;
//				return ;
//			}
//			chassis_terminal->manual_status = (manual_e)(chassis_zone2_adjust(chassis_terminal) + CHASSIS_GET_BALL7 - CHASSIS_GET_BALL1);
//		}
//		
//		if(xQueueReceive(RobotToChassisHandle,&rx_buff,0) == pdTRUE)
//		{
//			chassis_terminal->manual_status = (manual_e)rx_buff;
//		}
//	}
//	else if(chassis_terminal->chassis_RC->remote.sw1 == SW_DOWN )
//	{
//		//ȫ�Զ�
//		chassis_terminal->control_mode = CHASSIS_AUTO;
//		
//		if(xQueueReceive(RobotToChassisHandle,&rx_buff,0) == pdTRUE)
//		{
//			chassis_terminal->auto_status = (auto_e)rx_buff;
//		}
//	}
//	else
//	{
//		chassis_terminal->control_mode = CHASSIS_CMD_ERROR;
//	}
	
//	//�����볡״̬�л�
//	if(chassis_terminal->chassis_RC->remote.zone == CHASSIS_RED)
//	{
//		if(chassis_terminal->manual_status >= CHASSIS_GET_BALL1 && chassis_terminal->manual_status <= CHASSIS_SHOOT_BALL12)
//		{
//			chassis_terminal->manual_status += (manual_e)(CHASSIS_GET_BALL1_RED - CHASSIS_GET_BALL1);
//		}
//		if(chassis_terminal->auto_status >= GET_SEEDLING1 && chassis_terminal->auto_status <= DT35_CORRECT)
//		{
//			chassis_terminal->auto_status += (auto_e)(GET_SEEDLING1_RED - GET_SEEDLING1);
//		}
//	}
//	if(chassis_terminal->chassis_RC->remote.zone == CHASSIS_BLUE)
//	{
//		if(chassis_terminal->manual_status >= CHASSIS_GET_BALL1_RED && chassis_terminal->manual_status <= CHASSIS_SHOOT_BALL12_RED)
//		{
//			chassis_terminal->manual_status -= (manual_e)(CHASSIS_GET_BALL1_RED - CHASSIS_GET_BALL1);
//		}
//		if(chassis_terminal->auto_status >= GET_SEEDLING1_RED && chassis_terminal->auto_status <= DT35_CORRECT_RED)
//		{
//			chassis_terminal->auto_status -= (auto_e)(GET_SEEDLING1_RED - GET_SEEDLING1);
//		}
//	}
}

/**
  * @brief          ��ҡ���õ��̿�������ֵ
  * @param[out]     chassis_move_control:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{	
	  if (chassis_move_control == NULL)
    {
			return;
    }
		
    fp32 vx = 0.0f, vy = 0.0f, wz = 0.0f;
    int16_t vx_channel, vy_channel, wz_channel;
    //fp32 car_vel_set,car_move_angle_set; //����������ٶȺͷ��򣬺����Զ�·�����ܻ��õ�

    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
		//��Ϊx����������Ϊy��������˳ʱ��Ϊ��ת������
		
    rc_deadband_limit(-chassis_move_control->chassis_RC->ch[1], vx_channel, CHASSIS_RC_XY_DEADLINE);//ң����ԭʼֵ���Ϊ������
    rc_deadband_limit(chassis_move_control->chassis_RC->ch[2], vy_channel, CHASSIS_RC_XY_DEADLINE);
    rc_deadband_limit(chassis_move_control->chassis_RC->ch[0], wz_channel, CHASSIS_RC_WZ_DEADLINE);

    vx = vx_channel * 3000 / 660;
    vy = vy_channel * 3000 / 660;
		
//		abs_minus(wz_channel , wz_channel, 300);     
//  	wz = wz_channel * 3000 / 660;  
		
/**********************************************new����remote***********************************************************************/		
//    rc_deadband_limit(-chassis_move_control->chassis_RC->remote.ch3, vx_channel, CHASSIS_RC_XY_DEADLINE);
//    rc_deadband_limit(chassis_move_control->chassis_RC->remote.ch1, vy_channel, CHASSIS_RC_XY_DEADLINE);
//    //rc_deadband_limit(chassis_move_control->chassis_RC->ch[0], wz_channel, CHASSIS_RC_WZ_DEADLINE);
//		if(chassis_move_control->chassis_RC->remote.key12 == SINGLE_CLICK)
//		{
//			wz_channel = 660;
//		}
//		else if(chassis_move_control->chassis_RC->remote.key11 == SINGLE_CLICK)
//		{
//			wz_channel = -660;
//		}
//		else
//		{
//			wz_channel = 0;
//		}
//		
		
//    vx = vx_channel  * 2000 / 660;
//    vy = vy_channel * 2000 / 660;
//  	wz = wz_channel * 2000 / 660;  
//		
		//���趨�ٶ��˲�
		first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_vx,vx);
		first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_vy,vy);
		first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_wz,wz);
		//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_channel < CHASSIS_RC_XY_DEADLINE && vx_channel > -CHASSIS_RC_XY_DEADLINE )
    {
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
    }
    if (vy_channel < CHASSIS_RC_XY_DEADLINE && vy_channel > -CHASSIS_RC_XY_DEADLINE )
    {
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		if (wz_channel < CHASSIS_RC_WZ_DEADLINE && wz_channel > -CHASSIS_RC_WZ_DEADLINE )
    {
        chassis_move_control->chassis_cmd_slow_set_wz.out = 0.0f;
    }
		vx=chassis_move_control->chassis_cmd_slow_set_vx.out;
		vy=chassis_move_control->chassis_cmd_slow_set_vy.out;		
		wz=chassis_move_control->chassis_cmd_slow_set_wz.out;
		
		chassis_move_control->vx_set=vx;
		chassis_move_control->vy_set=vy; 
		chassis_move_control->wz_set=wz;
		if(chassis_move_control->chassis_RC->sw[RC_SW_L_CHANNEL] == RC_SW_UP && chassis_move_control->chassis_RC->sw[RC_SW_R_CHANNEL] == RC_SW_UP)
		{
			chassis_key_handle_zone2(chassis_move_control);		
		}
		else if(chassis_move_control->chassis_RC->sw[RC_SW_L_CHANNEL] == RC_SW_UP && chassis_move_control->chassis_RC->sw[RC_SW_R_CHANNEL] == RC_SW_MID)
		{
			chassis_key_handle_zone1(chassis_move_control);		
		}
		
		if (chassis_move_control->wz_set != 0)  
		{
			chassis_move_control->car_yaw_set=chassis_move_control->ops.car_total_yaw;
		}
		
	switch(chassis_move_control->zero_set)
	{
		case CHASSIS_ZERO:
			chassis_move_control->car_yaw_set = 0;
			break;
		case CHASSIS_LEFT_90:
      chassis_move_control->car_yaw_set = 90;
		break;
		case CHASSIS_RIGHT_90:
      chassis_move_control->car_yaw_set = -90;
			break;
		case NULL_CONTROL:
			break;
	}
		
		
		
//		if(chassis_move_control->zero_set == 1)
//		{
//			chassis_move_control->car_yaw_set = 0;
//		}
		chassis_move_control->yaw_correct_pid.output = 0;
		chassis_move_control->yaw_correct_pid.target = chassis_move_control->car_yaw_set;                                                                                       
		chassis_move_control->yaw_correct_pid.output = chassis_move_control->yaw_correct_pid.f_cal_pid(&chassis_move_control->yaw_correct_pid, chassis_move_control->ops.car_total_yaw);	
		chassis_move_control->wz_set+=chassis_move_control->yaw_correct_pid.output;
}	

/**
 * @brief          �����ܵ�Ŀ��λ����Ҫ���趨�ٶ�
 * @param[in]      chassis_move_control:"chassis_move"����ָ��.
 * @retval         none
 */
void chassis_auto_position(chassis_move_t *chassis_control) //�Զ�
{
/***********************************�����,x���꣬y���� pid����*******************************************/

  chassis_control->yaw_pid.output = 0;
  chassis_control->yaw_pid.target = chassis_control->car_yaw_set;                                                                                       
  chassis_control->yaw_pid.output=chassis_control->yaw_pid.f_cal_pid(&chassis_control->yaw_pid, chassis_control->ops.car_total_yaw);
	
	chassis_control->pos_x_pid.output = 0;
  chassis_control->pos_x_pid.target = chassis_control->pos_x_target;                                                                                       
  chassis_control->pos_x_pid.output=chassis_control->pos_x_pid.f_cal_pid(&chassis_control->pos_x_pid, chassis_control->ops.pos_x);
	
	chassis_control->pos_y_pid.output = 0;
  chassis_control->pos_y_pid.target = chassis_control->pos_y_target;                                                                                       
  chassis_control->pos_y_pid.output=chassis_control->pos_y_pid.f_cal_pid(&chassis_control->pos_y_pid, chassis_control->ops.pos_y);
	
	
	chassis_control->vx_set=chassis_control->pos_x_pid.output;
	chassis_control->vy_set=chassis_control->pos_y_pid.output;
	chassis_control->wz_set=chassis_control->yaw_pid.output;
	
}

///**
// * @brief          ���̿��� ·������ �ٶȻ�
// * @param[in]      chassis_move_control:"chassis_move"����ָ��.
// * @retval         none
// */
//void chassis_auto_speed(chassis_move_t *chassis_control) 
//{
//	
//	if(path_flow_speed(chassis_control,get_seedling1_path,GET_SEEDLING1_PATH_LEN) == 0)
//	{
//		//û���յ�
//		float sin_ang = sin(  DEG2R(chassis_control->car_dis_set)); 
//		float cos_ang = cos(  DEG2R(chassis_control->car_dis_set));
//		
//		chassis_control->yaw_pid.output = 0;
//		chassis_control->yaw_pid.target = chassis_control->car_yaw_set;                                                                                       
//		chassis_control->yaw_pid.output=chassis_control->yaw_pid.f_cal_pid(&chassis_control->yaw_pid, chassis_control->ops.car_total_yaw);
//	
//		chassis_control->vx_set=chassis_control->car_vel_set * sin_ang;
//		chassis_control->vy_set=chassis_control->car_vel_set * cos_ang;
//		chassis_control->wz_set=chassis_control->yaw_pid.output;
//	}
//	else
//	{
//		//���յ㣬��ʼ��ƫ
//		chassis_control->yaw_pid.output = 0;
//		chassis_control->yaw_pid.target = chassis_control->car_yaw_set;                                                                                       
//		chassis_control->yaw_pid.output=chassis_control->yaw_pid.f_cal_pid(&chassis_control->yaw_pid, chassis_control->ops.car_total_yaw);
//	
//		chassis_control->pos_x_pid.output = 0;
//		chassis_control->pos_x_pid.target = chassis_control->pos_x_target;                                                                                       
//		chassis_control->pos_x_pid.output=chassis_control->pos_x_pid.f_cal_pid(&chassis_control->pos_x_pid, chassis_control->ops.pos_x);
//		
//		chassis_control->pos_y_pid.output = 0;
//		chassis_control->pos_y_pid.target = chassis_control->pos_y_target;                                                                                       
//		chassis_control->pos_y_pid.output=chassis_control->pos_y_pid.f_cal_pid(&chassis_control->pos_y_pid, chassis_control->ops.pos_y);
//		
//		
//		chassis_control->vx_set=chassis_control->pos_x_pid.output;
//		chassis_control->vy_set=chassis_control->pos_y_pid.output;
//		chassis_control->wz_set=chassis_control->yaw_pid.output;
//	}
//}

/**
 * @brief          �趨��ҡ�����Զ�ģʽ�µ��ٶ�
 * @param[in]      chassis_move_control:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_manual_control(chassis_move_t *chassis_move_control)
{
		uint32_t tx_data = chassis_move_control->manual_status;
		ave_speed=(ABS(motor_chassis[0].speed_rpm) + ABS(motor_chassis[1].speed_rpm) + ABS(motor_chassis[2].speed_rpm) + ABS(motor_chassis[3].speed_rpm))/4;
		switch(chassis_move_control->manual_status)
		{
			//����ҡ
			case MANUAL:
				chassis_set_contorl(chassis_move_control); 
				break;
			//�������Զ�
			case SEMI_AUTO_READY:
				chassis_move_control->target = 0; //Ŀ��ֵ���㣬׼������һ��·��
				break;
			/*************************************���볡********************************************/
			case CHASSIS_GET_BALL1:
				if(path_flow_positon(chassis_move_control,get_ball1,GET_BALL1_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL1:
				if(path_flow_positon(chassis_move_control,shoot_ball1,SHOOT_BALL1_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_GET_BALL2:
				if(path_flow_positon(chassis_move_control,get_ball2,GET_BALL2_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL2:
				if(path_flow_positon(chassis_move_control,shoot_ball2,SHOOT_BALL2_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL3:
				if(path_flow_positon(chassis_move_control,get_ball3,GET_BALL3_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}				
				break;
			case CHASSIS_SHOOT_BALL3:
				if(path_flow_positon(chassis_move_control,shoot_ball3,SHOOT_BALL3_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL4:
				if(path_flow_positon(chassis_move_control,get_ball4,GET_BALL4_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL4:
				if(path_flow_positon(chassis_move_control,shoot_ball4,SHOOT_BALL4_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL5:
				if(path_flow_positon(chassis_move_control,get_ball5,GET_BALL5_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL5:
				if(path_flow_positon(chassis_move_control,shoot_ball5,SHOOT_BALL5_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL6:
				if(path_flow_positon(chassis_move_control,get_ball6,GET_BALL6_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL6:
				if(path_flow_positon(chassis_move_control,shoot_ball6,SHOOT_BALL6_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;					
			case CHASSIS_GET_BALL7:
				if(path_flow_positon(chassis_move_control,get_ball7,GET_BALL7_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL7:
				if(path_flow_positon(chassis_move_control,shoot_ball7,SHOOT_BALL7_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_GET_BALL8:
				if(path_flow_positon(chassis_move_control,get_ball8,GET_BALL8_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL8:
				if(path_flow_positon(chassis_move_control,shoot_ball8,SHOOT_BALL8_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL9:
				if(path_flow_positon(chassis_move_control,get_ball9,GET_BALL9_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL9:
				if(path_flow_positon(chassis_move_control,shoot_ball9,SHOOT_BALL9_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL10:
				if(path_flow_positon(chassis_move_control,get_ball10,GET_BALL10_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL10:
				if(path_flow_positon(chassis_move_control,shoot_ball10,SHOOT_BALL10_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL11:
				if(path_flow_positon(chassis_move_control,get_ball11,GET_BALL11_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL11:
				if(path_flow_positon(chassis_move_control,shoot_ball11,SHOOT_BALL11_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL12:
				if(path_flow_positon(chassis_move_control,get_ball12,GET_BALL12_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL12:	
				if(path_flow_positon(chassis_move_control,shoot_ball12,SHOOT_BALL12_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
				
				
				
			/*************************************��볡********************************************/
			case CHASSIS_GET_BALL1_RED:
				if(path_flow_positon(chassis_move_control,get_ball1_red,GET_BALL1_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL1_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball1_red,SHOOT_BALL1_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_GET_BALL2_RED:
				if(path_flow_positon(chassis_move_control,get_ball2_red,GET_BALL2_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL2_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball2_red,SHOOT_BALL2_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL3_RED:
				if(path_flow_positon(chassis_move_control,get_ball3_red,GET_BALL3_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}				
				break;
			case CHASSIS_SHOOT_BALL3_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball3_red,SHOOT_BALL3_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL4_RED:
				if(path_flow_positon(chassis_move_control,get_ball4_red,GET_BALL4_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL4_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball4_red,SHOOT_BALL4_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL5_RED:
				if(path_flow_positon(chassis_move_control,get_ball5_red,GET_BALL5_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL5_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball5_red,SHOOT_BALL5_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL6_RED:
				if(path_flow_positon(chassis_move_control,get_ball6_red,GET_BALL6_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL6_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball6_red,SHOOT_BALL6_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;					
			case CHASSIS_GET_BALL7_RED:
				if(path_flow_positon(chassis_move_control,get_ball7_red,GET_BALL7_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL7_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball7_red,SHOOT_BALL7_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_GET_BALL8_RED:
				if(path_flow_positon(chassis_move_control,get_ball8_red,GET_BALL8_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL8_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball8_red,SHOOT_BALL8_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL9_RED:
				if(path_flow_positon(chassis_move_control,get_ball9_red,GET_BALL9_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL9_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball9_red,SHOOT_BALL9_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL10_RED:
				if(path_flow_positon(chassis_move_control,get_ball10_red,GET_BALL10_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL10_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball10_red,SHOOT_BALL10_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL11_RED:
				if(path_flow_positon(chassis_move_control,get_ball11_red,GET_BALL11_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL11_RED:
				if(path_flow_positon(chassis_move_control,shoot_ball11_red,SHOOT_BALL11_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			case CHASSIS_GET_BALL12_RED:
				if(path_flow_positon(chassis_move_control,get_ball12_red,GET_BALL12_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = SEMI_AUTO_READY;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case CHASSIS_SHOOT_BALL12_RED:	
				if(path_flow_positon(chassis_move_control,shoot_ball12_red,SHOOT_BALL12_PATH_LEN) == 1)
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->manual_status = MANUAL;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;	
			default:
				break;
		}
}


/**
 * @brief          �趨ȫ�Զ�ģʽ�µ��ٶ�
 * @param[in]      chassis_move_control:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_auto_control(chassis_move_t *chassis_move_control)
{
		uint32_t tx_data = chassis_move_control->auto_status;
		ave_speed=(ABS(motor_chassis[0].speed_rpm) + ABS(motor_chassis[1].speed_rpm) + ABS(motor_chassis[2].speed_rpm) + ABS(motor_chassis[3].speed_rpm))/4;
		float new_x;
		float new_y;
		switch(chassis_move_control->auto_status)
		{
			case AUTO_READY:
				chassis_move_control->target = 0; //Ŀ��ֵ���㣬׼������һ��·��
				break;
			//һ��
			/*************************************���볡********************************************/
			case GET_SEEDLING1:
				if(path_flow_positon(chassis_move_control,get_seedling1_path,GET_SEEDLING1_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test1++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case GET_SEEDLING2:
				if(path_flow_positon(chassis_move_control,get_seedling2_path,GET_SEEDLING2_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test2++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}			
				break;
			case PUT_SEEDLING1:
				if(path_flow_positon(chassis_move_control,put_seedling1_path,PUT_SEEDLING1_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test3++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case PUT_SEEDLING2:
				if(path_flow_positon(chassis_move_control,put_seedling2_path,PUT_SEEDLING2_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test4++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}		
				break;
			case PUT_SEEDLING3:
				if(path_flow_positon(chassis_move_control,put_seedling3_path,PUT_SEEDLING3_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test5++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}		
				break;
			case PUT_SEEDLING4:
				if(path_flow_positon(chassis_move_control,put_seedling4_path,PUT_SEEDLING4_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test6++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}		
				break;
				
				
				
				
			case GET_SEEDLING3:
				if(path_flow_positon(chassis_move_control,get_seedling3_path,GET_SEEDLING3_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test7++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case GET_SEEDLING4:	
				if(path_flow_positon(chassis_move_control,get_seedling4_path,GET_SEEDLING4_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test8++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case PUT_SEEDLING5:
				if(path_flow_positon(chassis_move_control,put_seedling5_path,PUT_SEEDLING5_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test9++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}	
				break;
			case PUT_SEEDLING6:
				if(path_flow_positon(chassis_move_control,put_seedling6_path,PUT_SEEDLING6_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test10++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}	
				break;
			case PUT_SEEDLING7:
				if(path_flow_positon(chassis_move_control,put_seedling7_path,PUT_SEEDLING7_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test11++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}	
				break;
			case PUT_SEEDLING8:	
				if(path_flow_positon(chassis_move_control,put_seedling8_path,PUT_SEEDLING8_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test12++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}					
				break;
			
			//�ڶ���ͨ��DT35��ƫ
			case GOTO_ZONE2:
				if(path_flow_positon(chassis_move_control,goto_zone2,GOTO_ZONE2_PATH_LEN) == 1 )
				{
					chassis_move_control->auto_status = DT35_CORRECT;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case DT35_CORRECT:
				//@TODO:���¿��Է�װ��һ������
				dt35_send();
				vTaskDelay(50);
				new_x =  CAR_X_OFFSET - CHASSIS_LEN_HALF -dt35_read(&dt35[0]);
				new_y =  (MAP_Y_BOARD - dt35_read(&dt35[2]) - CAR_Y_OFFSET - CHASSIS_LEN_HALF);
				Update_X(new_x);
				Update_Y(new_y);
				xQueueSend(ChassisToRobotHandle,&tx_data,0);
				chassis_move_control->target = 0;
				break;
			/*************************************��볡********************************************/
			case GET_SEEDLING1_RED:
				if(path_flow_positon(chassis_move_control,get_seedling1_path_red,GET_SEEDLING1_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test1++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case GET_SEEDLING2_RED:
				if(path_flow_positon(chassis_move_control,get_seedling2_path_red,GET_SEEDLING2_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test2++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}			
				break;
			case PUT_SEEDLING1_RED:
				if(path_flow_positon(chassis_move_control,put_seedling1_path_red,PUT_SEEDLING1_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test3++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case PUT_SEEDLING2_RED:
				if(path_flow_positon(chassis_move_control,put_seedling2_path_red,PUT_SEEDLING2_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test4++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}		
				break;
			case PUT_SEEDLING3_RED:
				if(path_flow_positon(chassis_move_control,put_seedling3_path_red,PUT_SEEDLING3_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test5++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}		
				break;
			case PUT_SEEDLING4_RED:
				if(path_flow_positon(chassis_move_control,put_seedling4_path_red,PUT_SEEDLING4_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test6++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}		
				break;
			case GET_SEEDLING3_RED:
				if(path_flow_positon(chassis_move_control,get_seedling3_path_red,GET_SEEDLING3_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test7++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case GET_SEEDLING4_RED:	
				if(path_flow_positon(chassis_move_control,get_seedling4_path_red,GET_SEEDLING4_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test8++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case PUT_SEEDLING5_RED:
				if(path_flow_positon(chassis_move_control,put_seedling5_path_red,PUT_SEEDLING5_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test9++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}	
				break;
			case PUT_SEEDLING6_RED:
				if(path_flow_positon(chassis_move_control,put_seedling6_path_red,PUT_SEEDLING6_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test10++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}	
				break;
			case PUT_SEEDLING7_RED:
				if(path_flow_positon(chassis_move_control,put_seedling7_path_red,PUT_SEEDLING7_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test11++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}	
				break;
			case PUT_SEEDLING8_RED:	
				if(path_flow_positon(chassis_move_control,put_seedling8_path_red,PUT_SEEDLING8_PATH_LEN) == 1 )
				{
					xQueueSend(ChassisToRobotHandle,&tx_data,0);
					chassis_move_control->auto_status = AUTO_READY;
					chassis_test12++;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}					
				break;
			
			//�ڶ���ͨ��DT35��ƫ
			case GOTO_ZONE2_RED:
				if(path_flow_positon(chassis_move_control,goto_zone2_red,GOTO_ZONE2_PATH_LEN) == 1 )
				{
					chassis_move_control->auto_status = DT35_CORRECT_RED;
				}
				else
				{
					chassis_auto_position(chassis_move_control);
				}
				break;
			case DT35_CORRECT_RED: 
				//@TODO:���¿��Է�װ��һ������
				dt35_send();
				vTaskDelay(50);
				new_x =  (dt35_read(&dt35[1]) - CAR_X_OFFSET + CHASSIS_LEN_HALF);
				new_y =  (MAP_Y_BOARD - dt35_read(&dt35[2]) - CAR_Y_OFFSET - CHASSIS_LEN_HALF);
				Update_X(new_x);
				Update_Y(new_y);
				xQueueSend(ChassisToRobotHandle,&tx_data,0);
				chassis_move_control->target = 0;
				break;
			default:
				break;
		}
}


/**
 * @brief          ָ�����
 * @param[in]      chassis_move_control:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_cmd_error(chassis_move_t *chassis_control) 
{
	chassis_control->vx_set = 0;
	chassis_control->vy_set = 0;
	chassis_control->wz_set = 0;
	
	//chassis_control->target = 0;
	chassis_control->manual_status = MANUAL;
}

/**
 * @brief          ȫ���ֽ��� 
 * @param[in]      chassis_move_control:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_control) 
{
  float sin_ang = sin(  DEG2R(chassis_control->ops.car_total_yaw)); 
  float cos_ang = cos(  DEG2R(chassis_control->ops.car_total_yaw));
	
	chassis_control->wheel_velocity[0] =(-cos_ang + sin_ang) * chassis_control->vx_set +  (sin_ang + cos_ang) * chassis_control->vy_set + chassis_control->wz_set ;
	chassis_control->wheel_velocity[1] =(-cos_ang - sin_ang) * chassis_control->vx_set +  (sin_ang - cos_ang) * chassis_control->vy_set + chassis_control->wz_set;
	chassis_control->wheel_velocity[2] =( cos_ang - sin_ang) * chassis_control->vx_set +  (-sin_ang - cos_ang)  * chassis_control->vy_set + chassis_control->wz_set;
	chassis_control->wheel_velocity[3] =( cos_ang + sin_ang) * chassis_control->vx_set +  (-sin_ang + cos_ang) * chassis_control->vy_set + chassis_control->wz_set;
}

/**
  * @brief          ������ҡ״̬�µİ���ָ��
  * @param[out]     chassis_key_set ����ָ��.
  * @retval         none
  */
static void chassis_key_handle_zone1(chassis_move_t *chassis_key_set)
{
		if(chassis_key_set->game == MAIN)
		{
			switch(chassis_key_set->chassis_RC->mKey)
			{
				case KEY_L_R:
					chassis_key_set->zero_set = CHASSIS_RIGHT_90;
					break;
				case KEY_R_R:
					chassis_key_set->zero_set =  CHASSIS_LEFT_90;
					break;
				default:
					chassis_key_set->zero_set = NULL_CONTROL;
				break;
			}
		}
		if(chassis_key_set->game == SINGLE)
		{
			switch(chassis_key_set->chassis_RC->mKey)
			{
				case KEY_R_R:
					chassis_key_set->zero_set = CHASSIS_ZERO ;
					break;
				case KEY_R_L:
					chassis_key_set->zero_set = CHASSIS_RIGHT_90;
					break;
				default:
					chassis_key_set->zero_set = NULL_CONTROL;
				break;
			}
		}
}

/**
  * @brief          ������ҡ״̬�µİ���ָ��
  * @param[out]     chassis_key_set ����ָ��.
  * @retval         none
  */
static void chassis_key_handle_zone2(chassis_move_t *chassis_key_set)
{
//	if(shoot_key->zone == SHOOT_BLUE)
//	{
		switch(chassis_key_set->chassis_RC->mKey)
		{
			case KEY_L_R:
				chassis_key_set->zero_set = CHASSIS_RIGHT_90;
				break;
			case KEY_L_L:
				chassis_key_set->zero_set =  CHASSIS_LEFT_90;
				break;
			case KEY_R_R:
				chassis_key_set->zero_set = CHASSIS_ZERO ;
				break;
			default:
				chassis_key_set->zero_set = NULL_CONTROL;
			break;
		}
}

/************************ (C) COPYRIGHT 2024 EPOCH *****END OF FILE****/

