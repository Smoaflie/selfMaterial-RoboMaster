#include "usr_main.h"
#include "controller.h"
#include "motor.h"
#include "gyro.h"
#include "gimbal.h"
#include "chassis.h"

#include "freertos.h"
#include "cmsis_os2.h"
#include "task.h"

extern osThreadId_t InitTaskHandle;
extern osThreadId_t GimbalTaskHandle;
extern osThreadId_t ChassisTaskHandle;
extern osThreadId_t GyroTaskHandle;
extern osThreadId_t ControllerTaskHandle;

/* 陀螺仪任务 */
void _GyroTask(void *argument)
{
  /* USER CODE BEGIN _GyroTask */
  /* Infinite loop */
  for(;;)
  {
    gyro_task();

    // vTaskSuspend(NULL);  
    // osDelay(1);
  }
  /* USER CODE END _GyroTask */
}

/* 云台任务 */
void _GimbalTask(void *argument)
{
  /* USER CODE BEGIN _GimbalTask */
  /* Infinite loop */
  for(;;)
  {
    gimbal_task();
    
    vTaskSuspend(NULL);
    // osDelay(1);
  }
  /* USER CODE END _GimbalTask */
}

/* 控制器任务 */
void _ControllerTask(void *argument)
{
  /* USER CODE BEGIN _ControllerTask */
  /* Infinite loop */
  for(;;)
  {
    controller_task();

    vTaskResume(GimbalTaskHandle);
    vTaskResume(ChassisTaskHandle);
    osDelay(1);
    
  }
  /* USER CODE END _ControllerTask */
}

/* 底盘任务 */
void _ChassisTask(void *argument)
{
  /* USER CODE BEGIN _ChassisTask */
  /* Infinite loop */
  for(;;)
  {
    chassis_task();

    vTaskSuspend(NULL);
    // osDelay(1);
  }
  /* USER CODE END _ChassisTask */
}

/* 初始化任务 */
void _InitTask(void *argument)
{
  /* USER CODE BEGIN _InitTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_UART_Receive_DMA(&huart3, &RC_RxBuffer[0], 18);
    motor_config();
    gyro_init();
    gimbal_mainSet(); // 设定云台初始轴

    // vTaskSuspend(GyroTaskHandle);
    vTaskSuspend(GimbalTaskHandle);
    vTaskSuspend(ChassisTaskHandle);
    vTaskDelete(NULL);
  }
  /* USER CODE END _InitTask */
}