/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for InitTask */
osThreadId_t InitTaskHandle;
uint32_t InitTaskBuffer[ 128 ];
osStaticThreadDef_t InitTaskControlBlock;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .cb_mem = &InitTaskControlBlock,
  .cb_size = sizeof(InitTaskControlBlock),
  .stack_mem = &InitTaskBuffer[0],
  .stack_size = sizeof(InitTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
uint32_t GimbalTaskBuffer[ 128 ];
osStaticThreadDef_t GimbalTaskControlBlock;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .cb_mem = &GimbalTaskControlBlock,
  .cb_size = sizeof(GimbalTaskControlBlock),
  .stack_mem = &GimbalTaskBuffer[0],
  .stack_size = sizeof(GimbalTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
uint32_t ChassisTaskBuffer[ 128 ];
osStaticThreadDef_t ChassisTaskControlBlock;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .cb_mem = &ChassisTaskControlBlock,
  .cb_size = sizeof(ChassisTaskControlBlock),
  .stack_mem = &ChassisTaskBuffer[0],
  .stack_size = sizeof(ChassisTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GyroTask */
osThreadId_t GyroTaskHandle;
uint32_t GyroTaskBuffer[ 128 ];
osStaticThreadDef_t GyroTaskControlBlock;
const osThreadAttr_t GyroTask_attributes = {
  .name = "GyroTask",
  .cb_mem = &GyroTaskControlBlock,
  .cb_size = sizeof(GyroTaskControlBlock),
  .stack_mem = &GyroTaskBuffer[0],
  .stack_size = sizeof(GyroTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ControllerTask */
osThreadId_t ControllerTaskHandle;
uint32_t ChontrollerTaskBuffer[ 128 ];
osStaticThreadDef_t ChontrollerTaskControlBlock;
const osThreadAttr_t ControllerTask_attributes = {
  .name = "ControllerTask",
  .cb_mem = &ChontrollerTaskControlBlock,
  .cb_size = sizeof(ChontrollerTaskControlBlock),
  .stack_mem = &ChontrollerTaskBuffer[0],
  .stack_size = sizeof(ChontrollerTaskBuffer),
  .priority = (osPriority_t) osPriorityLow1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void _InitTask(void *argument);
void _GimbalTask(void *argument);
void _ChassisTask(void *argument);
void _GyroTask(void *argument);
void _ControllerTask(void *argument);

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
  /* creation of InitTask */
  InitTaskHandle = osThreadNew(_InitTask, NULL, &InitTask_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(_GimbalTask, NULL, &GimbalTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(_ChassisTask, NULL, &ChassisTask_attributes);

  /* creation of GyroTask */
  GyroTaskHandle = osThreadNew(_GyroTask, NULL, &GyroTask_attributes);

  /* creation of ControllerTask */
  ControllerTaskHandle = osThreadNew(_ControllerTask, NULL, &ControllerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header__InitTask */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header__InitTask */
__weak void _InitTask(void *argument)
{
  /* USER CODE BEGIN _InitTask */
  /* Infinite loop */
  for(;;)
  {
    
    osDelay(1);
  }
  /* USER CODE END _InitTask */
}

/* USER CODE BEGIN Header__GimbalTask */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__GimbalTask */
__weak void _GimbalTask(void *argument)
{
  /* USER CODE BEGIN _GimbalTask */
  /* Infinite loop */
  for(;;)
  {
    
    osDelay(1);
  }
  /* USER CODE END _GimbalTask */
}

/* USER CODE BEGIN Header__ChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__ChassisTask */
__weak void _ChassisTask(void *argument)
{
  /* USER CODE BEGIN _ChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _ChassisTask */
}

/* USER CODE BEGIN Header__GyroTask */
/**
* @brief Function implementing the GyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__GyroTask */
__weak void _GyroTask(void *argument)
{
  /* USER CODE BEGIN _GyroTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _GyroTask */
}

/* USER CODE BEGIN Header__ControllerTask */
/**
* @brief Function implementing the ControllerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__ControllerTask */
__weak void _ControllerTask(void *argument)
{
  /* USER CODE BEGIN _ControllerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _ControllerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

