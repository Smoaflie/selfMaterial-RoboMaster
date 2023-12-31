#include "usr_main.h"
#include "controller.h"
#include "motor.h"
#include "gyro.h"
#include "gimbal.h"
#include "chassis.h"

int usr_main(void)
{
    
    while(1);
}

/* 控制器数据接收中断 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_RxCpltCallback could be implemented in the user file
     */
    if (huart == &huart3) {
        RC_RecevieAnalysis(&RC_RxBuffer[0]);
        HAL_UART_Receive_DMA(&huart3, &RC_RxBuffer[0], 18);
    }
}

/* 电机数据接收中断 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef CAN_Rx_Message; // 定义接收数据结构体，用于存放IDE，RTR，DLC等内容
    uint8_t CAN_Rx_Data[8];             // 用于存储报文中的数据段

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Rx_Message, CAN_Rx_Data);
    if ((CAN_Rx_Message.IDE == CAN_ID_STD) && (CAN_Rx_Message.RTR == CAN_RTR_DATA) && (CAN_Rx_Message.DLC == 8)) {
        if (hcan == &hcan1) {
            motor_dataHandle(CAN_Rx_Message.StdId - 0x200, CAN_Rx_Data);
        }
    }
}
