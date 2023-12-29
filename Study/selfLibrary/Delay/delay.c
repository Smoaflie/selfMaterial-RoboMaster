#include "delay.h"
//这里使用的定时器为168Mhz，168-1分频，自动重载值为10-1，即10us触发一次
uint32_t volatile TIM11_uwTick=0;

static uint32_t TIM11_GetTick(void){
    return TIM11_uwTick;
}
void delay_1ms(uint32_t number){
    uint32_t tickstart = TIM11_GetTick();
    uint32_t wait = number;

    while(TIM11_GetTick() - tickstart < wait*100);
}
void delay_10us(uint32_t number){
    uint32_t tickstart = TIM11_GetTick();
    uint32_t wait = number;

    while(TIM11_GetTick() - tickstart < wait);
}

void TIM11_IncTick(void){
    TIM11_uwTick++;
}