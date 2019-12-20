/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
osThreadId Task_LED0Handle;
osThreadId TASK_LED1Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void Func_LED0(void const * argument);
void Func_LED1(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_LED0 */
  osThreadDef(Task_LED0, Func_LED0, osPriorityNormal, 0, 128);
  Task_LED0Handle = osThreadCreate(osThread(Task_LED0), NULL);

  /* definition and creation of TASK_LED1 */
  osThreadDef(TASK_LED1, Func_LED1, osPriorityIdle, 0, 128);
  TASK_LED1Handle = osThreadCreate(osThread(TASK_LED1), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Func_LED0 */
/**
  * @brief  Function implementing the Task_LED0 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Func_LED0 */
void Func_LED0(void const * argument)
{

  /* USER CODE BEGIN Func_LED0 */
  /* Infinite loop */
  for(;;)
  {

	printf("Task0 is running\r\n");

    osDelay(1000);
  }
  /* USER CODE END Func_LED0 */
}

/* USER CODE BEGIN Header_Func_LED1 */
/**
* @brief Function implementing the TASK_LED1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Func_LED1 */
void Func_LED1(void const * argument)
{
  /* USER CODE BEGIN Func_LED1 */
  /* Infinite loop */
  for(;;)
  {
	  printf("Task1 is running\r\n");
 
	  osDelay(1000);
  }
  /* USER CODE END Func_LED1 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// idle hook must be enable in cube
/* Declare a variable that will be incremented by the hook function. */
u8 ulIdleCycleCount = 0UL;
/* 空闲钩子函数必须命名为vApplicationIdleHook(),无参数也无返回值。 */
void vApplicationIdleHook( void )
{
/* This hook function does nothing but increment a counter. */
	ulIdleCycleCount++;
	printf("ulIdleCycleCount = %d\r\n",ulIdleCycleCount);
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
