#include "usr_main.h"

#include "LED.h"
#include "buzzer.h"
#include "bsp_usart.h"

int usr_main(void)
{
    USART_start();

    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start_IT(&htim11);
    LED_Color_Set('G');
    LED_freq_set(1000);
    buzzer_freq_set(1);

    while (1) {
      if(!HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)){
        buzzer_sone();
      }
    }
}

/* 定时器中断 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
     */
    if (htim == &htim11) {
        LED_freq_control();
        LED_lumx_control();
    }
}

/* CAN发送 */
void CAN_DataSent(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *CAN_Tx_Message, uint8_t *tx_buf)
{

    // CAN_TxHeaderTypeDef CAN_Tx_Message;//定义发送数据结构体，用于存放IDE，RTR，DLC等内容
    // CAN_Tx_Message.DLC    = 8;                //数据长度为8
    // CAN_Tx_Message.IDE    = CAN_ID_STD;       //数据为标准格式
    // CAN_Tx_Message.RTR    = CAN_RTR_DATA;     //代表为数据帧
    // CAN_Tx_Message.StdId  = StdId;

    if (hcan->State == HAL_CAN_STATE_READY || hcan->State == HAL_CAN_STATE_LISTENING) // 判断发送邮箱中是否存在空邮箱
    {
        HAL_CAN_AddTxMessage(hcan, CAN_Tx_Message, tx_buf, (uint32_t *)CAN_TX_MAILBOX2); // 将自定义报文添加到邮箱中
    }
}