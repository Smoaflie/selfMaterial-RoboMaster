#ifndef __DELAY_H
#define __DEALY_H
#include "main.h"

/*用户调用*/
void delay_1ms(uint32_t number);
void delay_10us(uint32_t number);

/*放在定时器中断内部*/
void TIM2_IncTick(void);
#endif  //__DEALY_H