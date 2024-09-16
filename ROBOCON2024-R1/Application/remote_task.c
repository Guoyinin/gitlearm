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
#include "remote_task.h"
#include "nrf24l01function.h"
#include "nrf24l01.h"
#include "chassis_task.h"
#include "plant_task.h"
#include "shoot_task.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

robot_status_t robot_status;


rc_info1_t rc_info;
nrf24l01_t rx_nrf24l01;
bsp_nrf24l01_t bsp_rx_nrf24l01 = 
{
	.bsp_nrf24l01_spi              = &hspi1,
	
	.bsp_nrf24l01_chip_select_port = RX_NRF24L01_CHIP_SELECT_GPIO_Port,
	.bsp_nrf24l01_chip_select_pin  = RX_NRF24L01_CHIP_SELECT_Pin,
	
	.bsp_nrf24l01_chip_enable_port = RX_NRF24L01_CHIP_ENABLE_GPIO_Port,
	.bsp_nrf24l01_chip_enable_pin  = RX_NRF24L01_CHIP_ENABLE_Pin,
	
	.bsp_nrf24l01_chip_iqr_port    = RX_NRF24L01_CHIP_IQR_GPIO_Port,
	.bsp_nrf24l01_chip_iqr_pin     = RX_NRF24L01_CHIP_IQR_Pin
};

nrf24l01_t tx_nrf24l01;
bsp_nrf24l01_t bsp_tx_nrf24l01 = 
{
	.bsp_nrf24l01_spi              = &hspi2,
	
	.bsp_nrf24l01_chip_select_port = TX_NRF24L01_CHIP_SELECT_GPIO_Port,
	.bsp_nrf24l01_chip_select_pin  = TX_NRF24L01_CHIP_SELECT_Pin,
																	 
	.bsp_nrf24l01_chip_enable_port = TX_NRF24L01_CHIP_ENABLE_GPIO_Port,
	.bsp_nrf24l01_chip_enable_pin  = TX_NRF24L01_CHIP_ENABLE_Pin,
																	 
	.bsp_nrf24l01_chip_iqr_port    = TX_NRF24L01_CHIP_IQR_GPIO_Port,
	.bsp_nrf24l01_chip_iqr_pin     = TX_NRF24L01_CHIP_IQR_Pin
};


/* Extern   variables ---------------------------------------------------------*/
extern chassis_move_t chassis_move;
extern shoot_t shoot;
extern plant_t plant;//取苗机构结构体
/* Extern   function prototypes -----------------------------------------------*/
extern uint8_t NRF24L01ReadWrite(nrf24l01_t* nrf, uint8_t txData);
extern void NRF24L01ChipSelectf(nrf24l01_t* nrf, NRF24L01CSType cs);
extern void NRF24L01ChipEnablef(nrf24l01_t* nrf, NRF24L01CEType en);
extern uint8_t NRF24L01GetIRQf(nrf24l01_t* nrf);
extern void SetNRF24L01Mode(nrf24l01_t *nrf, NRF24L01ModeType mode);

/* Private  function prototypes -----------------------------------------------*/
static enum NRF24L01Error remote_init(void);
static void robot_status_update(robot_status_t *robo_status_send);

uint8_t test;
uint8_t test1;

/* Private  functions ---------------------------------------------------------*/
		
/* USER CODE BEGIN 4 */


void remote_task(void const * argument)
{
  test = remote_init();
	test1 = NRF24L01Initialization(&tx_nrf24l01, NRF24L01ReadWrite, NRF24L01ChipSelectf,
                       NRF24L01ChipEnablef, NRF24L01GetIRQf, HAL_Delay, bsp_tx_nrf24l01);
	vTaskDelay(20);
  while(1)
  {
		NRF24L01ReceivePacket(&tx_nrf24l01, rc_info.packed);
	  vTaskDelay(1);
		NRF24L01ReceivePacket(&rx_nrf24l01, robot_status.packed);
	  vTaskDelay(1);
		robot_status_update(&robot_status);
	  vTaskDelay(1);
  }	
}

static enum NRF24L01Error remote_init(void)
{
  enum NRF24L01Error flag =  NRF24L01Initialization(&rx_nrf24l01, NRF24L01ReadWrite, NRF24L01ChipSelectf,
                       NRF24L01ChipEnablef, NRF24L01GetIRQf, HAL_Delay, bsp_rx_nrf24l01);
	return flag;
}

static void robot_status_update(robot_status_t *robo_status_send)
{
	robo_status_send->robot_status.chassis_auto_status    = chassis_move.auto_status;
	robo_status_send->robot_status.chassis_manual_status  = chassis_move.manual_status;
	robo_status_send->robot_status.chassis_target         = chassis_move.target;
	robo_status_send->robot_status.chassis_x              = chassis_move.ops.pos_x;
	robo_status_send->robot_status.chassis_y              = chassis_move.ops.pos_y;
	robo_status_send->robot_status.chassis_yaw            = chassis_move.ops.car_total_yaw;
	robo_status_send->robot_status.plant_status           = plant.status;
	robo_status_send->robot_status.shoot_status           = shoot.status;
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const rc_info1_t *get_remote_point(void)
{
    return &rc_info;
}
/* USER CODE END 4 */

/************************ (C) COPYRIGHT 2024 EPOCH *****END OF FILE****/
