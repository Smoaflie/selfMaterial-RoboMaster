/*
 * @Author: Smoaflie
 * @Date: 2023-12-29
 * @LastEditors: Smoaflie
 * @LastEditTime: 2023-11-10 22:20:47
 * @Description: 
 */

#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "usr_main.h"

void CAN_FIleter_init(void);
void CAN_DataSent(CAN_HandleTypeDef* hcan,uint16_t* tx_buf);
#endif // !__BSP_CAN_H
