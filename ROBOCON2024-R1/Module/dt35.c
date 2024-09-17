#include "dt35.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "string.h"
#include "gpio.h"
 

uint8_t tx_data[]={0x01, 0x04, 0x00, 0x00, 0x00, 0x04, 0xF1, 0xC9, };
uint8_t rx_buff_data[data_lenth];
dt35_t dt35[4];

float current_data[4] = {0.0f};

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	
//	if(USART6 == huart->Instance)
//	{
//		dt35_data_parsing();
//	}
//	HAL_UART_Receive_IT(&dt35_huart, rx_buff_data, 30);

//}

/**
 * @brief dt35_usart 初始化
 */

void usart_init(void)
{
	HAL_UART_Receive_IT(&dt35_huart, rx_buff_data, 30);
}

/**
  * @brief  dt35串口数据解析
  * @retval none
  */
//485 1215  580 2335


float dt35_data_parsing(void)
{	
  for (int i = 0; i < 14; i++)
  {
    if (rx_buff_data[i] == 0x01 && rx_buff_data[i + 1] == 0x04)
    {
			current_data[0] = ( rx_buff_data[i + 4]  | rx_buff_data[i + 3] << 8) * 1.0 / 249;
			current_data[1] = ( rx_buff_data[i + 6]  | rx_buff_data[i + 5] << 8) * 1.0 / 249;
			current_data[2] = ( rx_buff_data[i + 8]  | rx_buff_data[i + 7] << 8) * 1.0 / 249;
			current_data[3] = ( rx_buff_data[i + 10] | rx_buff_data[i + 9] << 8) * 1.0 / 249;
			
//			current_data[0] = current_buff_data[0] * 1.0 / 249;
//			current_data[1] = current_buff_data[1] * 1.0 / 249;
//			current_data[2] = current_buff_data[2] * 1.0 / 249;
//			current_data[3] = current_buff_data[03] * 1.0 / 249;
			
			dt35[0].current = current_data[0];
			dt35[1].current = current_data[1];
			dt35[2].current = current_data[2];
			dt35[3].current = current_data[3];
			
			dt35[0].distance = (current_data[0] - 3.983f) * 136.81840f + 485.0f;
			dt35[1].distance = (current_data[1] - 3.951f) * 136.61338f + 485.0f ;
			dt35[2].distance = (current_data[2] - 3.983f) * 136.64751f + 485.0f;
			dt35[3].distance = (current_data[3] - 4.0f) * 500 ;

      memset(&rx_buff_data, '0', sizeof(rx_buff_data));	
//       memset(&current_data, '0', sizeof(current_data));	
			break;
    }
  }	
}

/**
  * @brief  dt35数据接口
  * @param  dt35结构体
  * @retval dt35测得的距离
  */
float dt35_read(dt35_t *dt35)
{
	return dt35->distance;
}

/**
  * @brief  要发送一次，模块才会返回一次数据
  * @retval none
  */
void dt35_send(void)
{
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&dt35_huart, tx_data, 8, 0xFFFF);
	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}
