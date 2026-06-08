/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
osThreadId INS_TASKHandle;
osThreadId try_tryHandle;
osThreadId CHASSISS_CALCHandle;
osThreadId FSM_TASKHandle;
osThreadId GIMBAL_CALCHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void INS_Task(void const * argument);
void Try_Try(void const * argument);
void chassiss_calc(void const * argument);
void FSM_task(void const * argument);
void gimbal_calc(void const * argument);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_Task, osPriorityRealtime, 0, 128);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of try_try */
  osThreadDef(try_try, Try_Try, osPriorityNormal, 0, 128);
  try_tryHandle = osThreadCreate(osThread(try_try), NULL);

  /* definition and creation of CHASSISS_CALC */
  osThreadDef(CHASSISS_CALC, chassiss_calc, osPriorityIdle, 0, 128);
  CHASSISS_CALCHandle = osThreadCreate(osThread(CHASSISS_CALC), NULL);

  /* definition and creation of FSM_TASK */
  osThreadDef(FSM_TASK, FSM_task, osPriorityIdle, 0, 128);
  FSM_TASKHandle = osThreadCreate(osThread(FSM_TASK), NULL);

  /* definition and creation of GIMBAL_CALC */
  osThreadDef(GIMBAL_CALC, gimbal_calc, osPriorityIdle, 0, 128);
  GIMBAL_CALCHandle = osThreadCreate(osThread(GIMBAL_CALC), NULL);

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

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Try_Try */
/**
* @brief Function implementing the try_try thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Try_Try */
__weak void Try_Try(void const * argument)
{
  /* USER CODE BEGIN Try_Try */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Try_Try */
}

/* USER CODE BEGIN Header_chassiss_calc */
/**
* @brief Function implementing the CHASSISS_CALC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassiss_calc */
__weak void chassiss_calc(void const * argument)
{
  /* USER CODE BEGIN chassiss_calc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassiss_calc */
}

/* USER CODE BEGIN Header_FSM_task */
/**
* @brief Function implementing the FSM_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FSM_task */
__weak void FSM_task(void const * argument)
{
  /* USER CODE BEGIN FSM_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FSM_task */
}

/* USER CODE BEGIN Header_gimbal_calc */
/**
* @brief Function implementing the GIMBAL_CALC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_calc */
__weak void gimbal_calc(void const * argument)
{
  /* USER CODE BEGIN gimbal_calc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_calc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
