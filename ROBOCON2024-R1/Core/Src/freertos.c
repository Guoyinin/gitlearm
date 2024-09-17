/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId MotorTaskHandle;
osThreadId PlantTaskHandle;
osThreadId ShootTaskHandle;
osThreadId RobotTaskHandle;
osThreadId remoteTaskHandle;
osMessageQId RobotToChassisHandle;
osMessageQId RobotToPlantHandle;
osMessageQId ChassisToRobotHandle;
osMessageQId PlantToRobotHandle;
osMessageQId RobotToShootHandle;
osMessageQId ShootToRobotHandle;
osSemaphoreId RobotChassisSemphoreHandle;
osSemaphoreId RobotPlantSemphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void chassis_task(void const * argument);
extern void motor_task(void const * argument);
extern void plant_task(void const * argument);
extern void shoot_task(void const * argument);
extern void robot_task(void const * argument);
extern void remote_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of RobotChassisSemphore */
  osSemaphoreDef(RobotChassisSemphore);
  RobotChassisSemphoreHandle = osSemaphoreCreate(osSemaphore(RobotChassisSemphore), 1);

  /* definition and creation of RobotPlantSemphore */
  osSemaphoreDef(RobotPlantSemphore);
  RobotPlantSemphoreHandle = osSemaphoreCreate(osSemaphore(RobotPlantSemphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of RobotToChassis */
  osMessageQDef(RobotToChassis, 1, uint32_t);
  RobotToChassisHandle = osMessageCreate(osMessageQ(RobotToChassis), NULL);

  /* definition and creation of RobotToPlant */
  osMessageQDef(RobotToPlant, 1, uint32_t);
  RobotToPlantHandle = osMessageCreate(osMessageQ(RobotToPlant), NULL);

  /* definition and creation of ChassisToRobot */
  osMessageQDef(ChassisToRobot, 1, uint32_t);
  ChassisToRobotHandle = osMessageCreate(osMessageQ(ChassisToRobot), NULL);

  /* definition and creation of PlantToRobot */
  osMessageQDef(PlantToRobot, 1, uint32_t);
  PlantToRobotHandle = osMessageCreate(osMessageQ(PlantToRobot), NULL);

  /* definition and creation of RobotToShoot */
  osMessageQDef(RobotToShoot, 1, uint32_t);
  RobotToShootHandle = osMessageCreate(osMessageQ(RobotToShoot), NULL);

  /* definition and creation of ShootToRobot */
  osMessageQDef(ShootToRobot, 1, uint32_t);
  ShootToRobotHandle = osMessageCreate(osMessageQ(ShootToRobot), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, chassis_task, osPriorityNormal, 0, 256);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, motor_task, osPriorityNormal, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of PlantTask */
  osThreadDef(PlantTask, plant_task, osPriorityNormal, 0, 128);
  PlantTaskHandle = osThreadCreate(osThread(PlantTask), NULL);

  /* definition and creation of ShootTask */
  osThreadDef(ShootTask, shoot_task, osPriorityNormal, 0, 128);
  ShootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

  /* definition and creation of RobotTask */
  osThreadDef(RobotTask, robot_task, osPriorityNormal, 0, 256);
  RobotTaskHandle = osThreadCreate(osThread(RobotTask), NULL);

  /* definition and creation of remoteTask */
  osThreadDef(remoteTask, remote_task, osPriorityIdle, 0, 128);
  remoteTaskHandle = osThreadCreate(osThread(remoteTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
