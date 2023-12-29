#include "bsp_usart.h"

#define Rx1Buffer_max 1
#define Rx6Buffer_max 1
uint8_t Rx1Buffer[Rx1Buffer_max];
uint8_t Rx6Buffer[Rx6Buffer_max];

void USART_start(void){
    HAL_UART_Receive_DMA(&huart1, Rx1Buffer, Rx1Buffer_max);
    HAL_UART_Receive_DMA(&huart6, Rx6Buffer, Rx6Buffer_max);
}

/* 控制器数据接收中断 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_RxCpltCallback could be implemented in the user file
     */
    if(huart==&huart1){
        uint8_t pData;
        HAL_UART_Transmit_DMA(&huart6,Rx1Buffer,Rx1Buffer_max);
        HAL_UART_Receive_DMA(&huart1, Rx1Buffer, Rx1Buffer_max);
    }
    if(huart==&huart6){
        uint8_t pData;
        HAL_UART_Transmit_DMA(&huart1,Rx6Buffer,Rx6Buffer_max);
        HAL_UART_Receive_DMA(&huart6, Rx6Buffer, Rx6Buffer_max);
    }
}