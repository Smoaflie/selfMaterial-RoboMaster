/*
 * @Author: Smoaflie
 * @Date: 2023-12-29
 * @LastEditors: Smoaflie
 * @LastEditTime: 2023-11-10 22:20:47
 * @Description: 
 */
#ifndef __DELAY_H
#define __DEALY_H
#include "usr_main.h"

/*用户调用*/
void delay_1ms(uint32_t number);
void delay_10us(uint32_t number);

/*放在定时器中断内部*/
void TIM11_IncTick(void);
#endif  //__DEALY_H