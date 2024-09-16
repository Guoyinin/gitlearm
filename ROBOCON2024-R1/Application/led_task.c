/**
  ******************************************************************************
  * @file     led_task.c
  * @author   Junshuo
  * @version  V1.0
  * @date     Jun-28-2023
  * @brief    led任务，用来指示程序运行状态
  ******************************************************************************
  * @attention
  * 注意事项
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "led_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
#define LED_TASK_INIT_TIME 357
#define LED_CONTROL_TIME_MS 100
/* Private  variables ---------------------------------------------------------*/

/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/**
  * @brief  LedTask 简单点灯
  * @param  None
  * @retval None
  */
void led_task(void const *pvParameters)
{
  vTaskDelay(LED_TASK_INIT_TIME);
  
//  LED1_OFF();
//  LED2_OFF();
//  LED3_OFF();

	while(1)
	{
//    LED1_TOGGLE();
//    LED2_TOGGLE();
//    LED3_TOGGLE();
    
		vTaskDelay(LED_CONTROL_TIME_MS);

	}

}


/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/

