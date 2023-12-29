#include "bsp_CAN.h"

CAN_FilterTypeDef CAN1_FilterConfig,CAN2_FilterConfig;
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

    CAN2_FilterConfig.FilterActivation = ENABLE;           // 过滤器使能
    CAN2_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
    CAN2_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位
    CAN2_FilterConfig.FilterIdHigh = 0x0000;               // 过滤器ID
    CAN2_FilterConfig.FilterIdLow = 0x0000;
    CAN2_FilterConfig.FilterMaskIdHigh = 0x0000;
    CAN2_FilterConfig.FilterMaskIdLow = 0x0000;
    CAN2_FilterConfig.FilterBank = 0;
    CAN2_FilterConfig.SlaveStartFilterBank = 14;
    CAN2_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; // 关联FIFO0

    HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilterConfig);
    HAL_CAN_ConfigFilter(&hcan2, &CAN2_FilterConfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 打开FIFO待处理中断
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING); // 打开FIFO待处理中断
}
