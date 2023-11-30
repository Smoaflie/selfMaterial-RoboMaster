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
