#include "usr_main.h"
#include "controller.h"
#include "motor.h"
uint16_t  aRC_ROCKER_L = 0,    //左摇杆推度
    aRC_ROCKER_L_DEG,    //左摇杆角度
    aRC_ROCKER_R,        //右摇杆推度
    aRC_ROCKER_R_DEG,    //右摇杆角度
    aRC_PULLROD,         //拨杆状态
    aRC_KEY;              //按键
int usr_main(void){
    motor_config();
    HAL_UART_Receive_DMA(&huart3,&RC_RxBuffer[0],18);
    motor_rotate_speed_set(1,500);

    // ! NOTE:while死循环不能删除
    while(1){
        aRC_ROCKER_L = RC_GetData(RC_ROCKER_L);
        aRC_ROCKER_L_DEG= RC_GetData(RC_ROCKER_L_DEG);
        aRC_ROCKER_R= RC_GetData(RC_ROCKER_R);
        aRC_ROCKER_R_DEG= RC_GetData(RC_ROCKER_R_DEG);
        aRC_PULLROD= RC_GetData(RC_PULLROD);
        aRC_KEY= RC_GetData(RC_KEY);
        motor_rotate_speed_set(1,3000*aRC_ROCKER_L/100);
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