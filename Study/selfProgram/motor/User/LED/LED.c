#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "LED.h"

static uint8_t LED_Color='W';
    static uint8_t LED_lumx;
    static uint32_t LED_flicker_period;//(10us)
    static uint32_t limit_value=2000;//(10us)
//设置LED灯颜色
void LED_Color_Set(uint8_t Color){
    LED_Color=Color;
    //先全灭
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);
    //再显示对应颜色
    switch(LED_Color){
        case 'R':
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            break;
        case 'G':
            HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
            break;
        case 'B':
            HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
            break;
        case 'W':
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
            break;
        default:
            break;
    }
}
void LED_lumx_set(uint8_t lumx_t){
    LED_lumx=lumx_t;//按百分比控制亮度
}
void LED_freq_set(uint16_t freq){//闪烁周期，单位ms
    LED_flicker_period=freq*1000;
}
//放在定时器中断中，控制灯亮度
//本工程中使用的是1us定时器中断
void LED_lumx_control(void){//亮度，百分比(0-100)
    static uint8_t LED_State=0;//标志LED是亮还是灭
    static uint32_t count=0;
    uint32_t lumx=(uint16_t)LED_lumx*limit_value/100;//按百分比控制亮度
    count++;
    if(count<=lumx){
        LED_Color_Set(LED_Color);
        LED_State=1;
    }else if(count>=lumx){
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);
        LED_State=0;
    }
    if(count>=limit_value){
        count=0;
    }
}
//控制灯闪烁频率
void LED_freq_control(void){
    static uint32_t count=0;//计数
    static uint8_t lumx=0;  //亮度，从0到1再从1到0
    count++;
    if(count>=LED_flicker_period/200){
        count=0;
        lumx++;
        if(lumx>=200)    lumx=0;
    }
    if(lumx>=100){
        LED_lumx_set(100-lumx%100);
    }else{
        LED_lumx_set(lumx%100);
    }
}