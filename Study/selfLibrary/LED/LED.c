/*
 * @Author: Smoaflie
 * @Date: 2023-12-29
 * @LastEditors: Smoaflie
 * @LastEditTime: 2023-11-10 22:20:47
 * @Description:
 */
#include "LED.h"

#include "stdio.h"

#ifndef LED_R_Pin
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#endif

static uint8_t LED_Color = 'W';

static uint32_t LED_lumx; // 亮度

static uint16_t LED_period = 100;          // 周期
static uint16_t LED_flicker_period = 100;  // 非空比
static uint32_t LED_flicker_freq = 100000; // 闪烁频率

//设置LED灯颜色
void LED_Color_Set(uint8_t Color){
    LED_Color = Color;
}

void LED_On(void){
    //显示对应颜色
    switch(LED_Color){
        case 'R':
            HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);

            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            break;
        case 'G':
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);

            HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
            break;
        case 'B':
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);

            HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
            break;
        case 'W':
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
            break;
        default:
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);
            break;
    }
}
void LED_Off(void){
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);
}


/* 设置亮度 */
void LED_lumx_set(float lumx_t)
{
    // 精确到小数点后两位
    lumx_t = ((uint16_t)(lumx_t * 100) / 100.0);
    uint16_t period_t = 100;
    while (lumx_t - (uint16_t)lumx_t)
    {
        lumx_t *= 10;
        period_t *= 10;
    }
    LED_period = period_t;
    LED_lumx = (uint16_t)lumx_t; // 按百分比控制亮度
}

/* 设置闪烁频率 */
void LED_freq_set(uint32_t freq_t)
{
    LED_flicker_freq = freq_t;
}

//放在定时器中断中，控制灯亮度
void LED_lumx_control(void){//亮度，百分比(0-100)
    static uint32_t count=1;
    count++;
    if(count>=LED_period){
        count=1;
    }
    if(count<=LED_flicker_period){
        LED_On();
    }else if(count<=LED_period){
        LED_Off();
    }
}
//控制灯闪烁频率
void LED_freq_control(void){
    static uint32_t count=0;//计数
    static uint8_t lumx=0;  //亮度，从0到100再从100到0
    count++;
    if(count>=LED_flicker_freq){
        count=1;
        lumx++;
        if(lumx>=200)    lumx=0;

        LED_lumx_set(lumx>=100?(100-lumx%100):(lumx%100));
    }
}