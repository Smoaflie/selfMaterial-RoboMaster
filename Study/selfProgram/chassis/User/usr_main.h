#ifndef __USR_MAIN_H_
#define __USR_MAIN_H_

#include "main.h"

#include "usart.h"
#include "can.h"
#include "tim.h"

#include "freertos.h"
#include "cmsis_os2.h"
#include "task.h"

extern osThreadId_t InitTaskHandle;
extern osThreadId_t GimbalTaskHandle;
extern osThreadId_t ChassisTaskHandle;
extern osThreadId_t GyroTaskHandle;
extern osThreadId_t ControllerTaskHandle;

#define M_PI 3.1416

int usr_main(void);

#endif // !__USR_MAIN_H_