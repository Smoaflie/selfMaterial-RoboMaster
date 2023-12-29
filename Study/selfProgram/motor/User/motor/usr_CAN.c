/*
 * @Author: Smoaflie
 * @Date: 2023-11-08 20:23:50
 * @LastEditors: Smoaflie
 * @LastEditTime: 2023-11-10 22:23:10
 * @Description: 请填写简介
 */
#include "usr_CAN.h"
#include "motor.h"

CAN_FilterTypeDef CAN1_FilterConfig;
extern CAN_HandleTypeDef hcan1,hcan2;

void CAN_FIleter_init(void)
{
    CAN1_FilterConfig.FilterActivation = ENABLE;           // 过滤器使能
    CAN1_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
    CAN1_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位
    CAN1_FilterConfig.FilterIdHigh = 0x0000;               // 过滤器ID
    CAN1_FilterConfig.FilterIdLow = 0x0000;
    CAN1_FilterConfig.FilterMaskIdHigh = 0x0000;
    CAN1_FilterConfig.FilterMaskIdLow = 0x0000;
    CAN1_FilterConfig.FilterBank = 0;
    CAN1_FilterConfig.SlaveStartFilterBank = 14;
    CAN1_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 关联FIFO0
    HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilterConfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 打开FIFO待处理中断
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // static uint16_t time_out = 0;
    // if (time_out++ < 10000)
    //     return;
    CAN_RxHeaderTypeDef CAN_Rx_Message; // 定义接收数据结构体，用于存放IDE，RTR，DLC等内容
    uint8_t CAN_Rx_Data[8];              // 用于存储报文中的数据段

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Rx_Message, CAN_Rx_Data);
    if ((CAN_Rx_Message.IDE == CAN_ID_STD) && (CAN_Rx_Message.RTR == CAN_RTR_DATA) && (CAN_Rx_Message.DLC == 8))
    {
        /* 我们规定：can1控制电机编号1-8,can2控制电机编号9-16 */
        if(hcan == &hcan2)  CAN_Rx_Message.StdId+=8;

        // 这里写用户自定义数据解析函数
        motor_DataHandle(CAN_Rx_Message.StdId - 0x200, CAN_Rx_Data);
    }

}
