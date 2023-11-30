#include "usr_main.h"
#include "controller.h"
#include "motor.h"
#include "gyro.h"

uint16_t  aRC_ROCKER_L = 0,    //左摇杆推度
    aRC_ROCKER_L_DEG,    //左摇杆角度
    aRC_ROCKER_R,        //右摇杆推度
    aRC_ROCKER_R_DEG,    //右摇杆角度
    aRC_PULLROD,         //拨杆状态
    aRC_KEY;              //按键
int usr_main(void){
    gyro_init();
    motor_config();
    HAL_UART_Receive_DMA(&huart3,&RC_RxBuffer[0],18);

    // ! NOTE:while死循环不能删除
    while(1){
        
        // aRC_ROCKER_L = RC_GetData(RC_ROCKER_L);
        // aRC_ROCKER_L_DEG= RC_GetData(RC_ROCKER_L_DEG);
        // aRC_ROCKER_R= RC_GetData(RC_ROCKER_R);
        // aRC_ROCKER_R_DEG= RC_GetData(RC_ROCKER_R_DEG);
        // aRC_PULLROD= RC_GetData(RC_PULLROD);
        // aRC_KEY= RC_GetData(RC_KEY);
        // motor_rotate_speed_set(1,3000*aRC_ROCKER_L/100);
    }
    
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  RC_RecevieAnalysis(&RC_RxBuffer[0]);
  HAL_UART_Receive_DMA(&huart3,&RC_RxBuffer[0],18);
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