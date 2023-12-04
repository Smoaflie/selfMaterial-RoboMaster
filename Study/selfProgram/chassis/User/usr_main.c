#include "usr_main.h"
#include "controller.h"
#include "motor.h"
#include "gyro.h"
/*
    TODO:
    1.找机械要个板子，打孔，将6020转子与C板连接固定
    2.写代码，通过陀螺仪控制6020角度
    3.将这鬼东西装在底盘上，测试跟随等模式
    PTZ_Get()
    搜这个定位修改处，改为用陀螺仪获取云台方向`
*/
// uint16_t  aRC_ROCKER_L = 0,    //左摇杆推度
//     aRC_ROCKER_L_DEG,    //左摇杆角度
//     aRC_ROCKER_R,        //右摇杆推度
//     aRC_ROCKER_R_DEG,    //右摇杆角度
//     aRC_PULLROD,         //拨杆状态
//     aRC_KEY;              //按键
int usr_main(void){
    HAL_UART_Receive_DMA(&huart3,&RC_RxBuffer[0],18);
    motor_config();
    gyro_init();
    PTZ_mainSet();  //设定云台初始轴
    HAL_TIM_Base_Start_IT(&htim3);

    while(1){
        // static float l_v=0;
        // aRC_ROCKER_L = RC_GetData(RC_ROCKER_L);
        // aRC_ROCKER_L_DEG= RC_GetData(RC_ROCKER_L_DEG);
        // aRC_ROCKER_R= RC_GetData(RC_ROCKER_R);
        // aRC_ROCKER_R_DEG= RC_GetData(RC_ROCKER_R_DEG);
        // aRC_PULLROD= RC_GetData(RC_PULLROD);
        // aRC_KEY= RC_GetData(RC_KEY);
        // if(RC_GetData(RC_ROCKER_R)>50)  {motor_rotate_radian_set(5,aRC_ROCKER_R_DEG);l_v=aRC_ROCKER_R_DEG;gimbalFLAG=gimbal_ROTATE;}
        // else{motor_rotate_radian_set(5,l_v);gimbalFLAG=gimbal_HOLD;}
        RC_CtrlChassis();
        // if(aRC_ROCKER_L==100)   {PTZ_mainSet();motor_rotate_radian_set(5,0);l_v=0;while(RC_GetData(RC_ROCKER_R_DEG)!=0);}  //设定云台初始轴
        // if(aRC_PULLROD==0x88)   {gyro_setCurRadian();motor_rotate_radian_set(5,0);l_v=0;}   
    }
    
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
  if(htim==&htim3){
    gyro_calData();
    for(int i=1;i<6;i++){
        ElectricMotor *p_motor=&motor[i];
        if(p_motor->type==MotorSpeed)  motor_control_current_set(p_motor);
        else if(p_motor->type==MotorRadian)    motor_control_voltage_set(p_motor);
    }
    motor_can_send_control_current();
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

