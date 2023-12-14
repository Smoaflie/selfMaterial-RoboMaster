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

void CAN_DataSent(CAN_HandleTypeDef* hcan,uint16_t StdId,uint16_t data1,uint16_t data2,uint16_t data3,uint16_t data4){
    CAN_TxHeaderTypeDef CAN_Tx_Message;//定义发送数据结构体，用于存放IDE，RTR，DLC等内容
    uint8_t CAN_Tx_Data[8];                   //用于暂存发送报文中的数据段
    
    CAN_Tx_Message.DLC    = 8;                //数据长度为8
    CAN_Tx_Message.IDE    = CAN_ID_STD;       //数据为标准格式
    CAN_Tx_Message.RTR    = CAN_RTR_DATA;     //代表为数据帧
    CAN_Tx_Message.StdId  = StdId;
    CAN_Tx_Data[0] = data1 >>8;
    CAN_Tx_Data[1] = data1;    
    CAN_Tx_Data[2] = data2 >>8;
    CAN_Tx_Data[3] = data2;    
    CAN_Tx_Data[4] = data3 >>8;
    CAN_Tx_Data[5] = data3;    
    CAN_Tx_Data[6] = data4 >>8;
    CAN_Tx_Data[7] = data4;    

    if (hcan->State == HAL_CAN_STATE_READY || hcan->State == HAL_CAN_STATE_LISTENING) //判断发送邮箱中是否存在空邮箱
    {
        if (StdId == 0x200 && (hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
                HAL_CAN_AddTxMessage(hcan, &CAN_Tx_Message, CAN_Tx_Data, (uint32_t*)CAN_TX_MAILBOX0);//将自定义报文添加到邮箱中
        else if (StdId == 0x1ff && (hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
                HAL_CAN_AddTxMessage(hcan, &CAN_Tx_Message, CAN_Tx_Data, (uint32_t*)CAN_TX_MAILBOX1);//将自定义报文添加到邮箱中
        else if (StdId == 0x2ff && (hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
                HAL_CAN_AddTxMessage(hcan, &CAN_Tx_Message, CAN_Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);//将自定义报文添加到邮箱中  
    }
}