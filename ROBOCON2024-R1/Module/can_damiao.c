/**
  ******************************************************************************
  * @file     can_damiao.c
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
#include "can_damiao.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static CAN_TxHeaderTypeDef  damiao_tx_message;
static uint8_t              damiao_can_send_data[8];
 damiao_motor_t motor_over[1];

/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/**
  * @brief  
  * @param 
  * @param 
  * @retval 
  */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  // converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
int float_to_uint(float x, float x_min, float x_max, int bits)
{
  // Converts a float to an unsigned int, given range and number of bits///
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
	
	
	
void ctrl_motor(uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq)
{
  uint32_t send_mail_box;
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
  kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
  kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

  damiao_tx_message.StdId = id;
  damiao_tx_message.IDE = CAN_ID_STD;
  damiao_tx_message.RTR = CAN_RTR_DATA;
  damiao_tx_message.DLC = 0x08;
  damiao_can_send_data[0] = (pos_tmp >> 8);
  damiao_can_send_data[1] = pos_tmp;
  damiao_can_send_data[2] = (vel_tmp >> 4);
  damiao_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
  damiao_can_send_data[4] = kp_tmp;
  damiao_can_send_data[5] = (kd_tmp >> 4);
  damiao_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
  damiao_can_send_data[7] = tor_tmp;
  while ( !(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) ){} //等待空邮箱

  HAL_CAN_AddTxMessage(&DAMIAO_CAN, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}
 
 void ctrl_motor_in(uint16_t id)
{
  uint32_t send_mail_box;	 

  damiao_tx_message.StdId = id;
  damiao_tx_message.IDE = CAN_ID_STD;
  damiao_tx_message.RTR = CAN_RTR_DATA;
  damiao_tx_message.DLC = 0x08;
  damiao_can_send_data[0] = 0xFF;
  damiao_can_send_data[1] = 0xFF;
  damiao_can_send_data[2] = 0xFF;
  damiao_can_send_data[3] = 0xFF;
  damiao_can_send_data[4] = 0xFF;
  damiao_can_send_data[5] = 0xFF;
  damiao_can_send_data[6] = 0xFF;
  damiao_can_send_data[7] = 0xFC;
	
	while ( !(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) ){} //等待空邮箱

  HAL_CAN_AddTxMessage(&DAMIAO_CAN, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}
 
 
 
void ctrl_motor_out(uint16_t id)
{
  uint32_t send_mail_box;	 

  damiao_tx_message.StdId = id;
  damiao_tx_message.IDE = CAN_ID_STD;
  damiao_tx_message.RTR = CAN_RTR_DATA;
  damiao_tx_message.DLC = 0x08;
  damiao_can_send_data[0] = 0xFF;
  damiao_can_send_data[1] = 0xFF;
  damiao_can_send_data[2] = 0xFF;
  damiao_can_send_data[3] = 0xFF;
  damiao_can_send_data[4] = 0xFF;
  damiao_can_send_data[5] = 0xFF;
  damiao_can_send_data[6] = 0xFF;
  damiao_can_send_data[7] = 0xFD;
	while ( !(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) ){} //等待空邮箱
  HAL_CAN_AddTxMessage(&DAMIAO_CAN, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}

 
void ctrl_motor_zero(uint16_t id)
{
  uint32_t send_mail_box;	 

  damiao_tx_message.StdId = id;
  damiao_tx_message.IDE = CAN_ID_STD;
  damiao_tx_message.RTR = CAN_RTR_DATA;
  damiao_tx_message.DLC = 0x08;
  damiao_can_send_data[0] = 0xFF;
  damiao_can_send_data[1] = 0xFF;
  damiao_can_send_data[2] = 0xFF;
  damiao_can_send_data[3] = 0xFF;
  damiao_can_send_data[4] = 0xFF;
  damiao_can_send_data[5] = 0xFF;
  damiao_can_send_data[6] = 0xFF;
  damiao_can_send_data[7] = 0xFE;
	while ( !(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) ){} //等待空邮箱
  HAL_CAN_AddTxMessage(&DAMIAO_CAN, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}
 
void ctrl_motor_clear
  (uint16_t id)
{
  uint32_t send_mail_box;	 

  damiao_tx_message.StdId = id;
  damiao_tx_message.IDE = CAN_ID_STD;
  damiao_tx_message.RTR = CAN_RTR_DATA;
  damiao_tx_message.DLC = 0x08;
  damiao_can_send_data[0] = 0xFF;
  damiao_can_send_data[1] = 0xFF;
  damiao_can_send_data[2] = 0xFF;
  damiao_can_send_data[3] = 0xFF;
  damiao_can_send_data[4] = 0xFF;
  damiao_can_send_data[5] = 0xFF;
  damiao_can_send_data[6] = 0xFF;
  damiao_can_send_data[7] = 0xFB;
	while ( !(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) ){} //等待空邮箱
  HAL_CAN_AddTxMessage(&DAMIAO_CAN, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}


/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/

