#include "delay.h"
//这里使用的定时器为84Mhz，84-1分频，自动重载值为10-1，即10us触发一次
uint32_t TIM2_uwTick=0;

static uint32_t TIM2_GetTick(void){
    return TIM2_uwTick;
}
void delay_1ms(uint32_t number){
    uint32_t tickstart = TIM2_GetTick();
    uint32_t wait = number;

    while(TIM2_GetTick() - tickstart < wait*100);
}
void delay_10us(uint32_t number){
    uint32_t tickstart = TIM2_GetTick();
    uint32_t wait = number;

    while(TIM2_GetTick() - tickstart < wait);
}

void TIM2_IncTick(void){
    TIM2_uwTick++;
}