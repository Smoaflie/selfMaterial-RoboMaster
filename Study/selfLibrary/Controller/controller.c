/* 除了处理遥控器数据，将DMA双缓冲设置也放到这个文件里了 */
#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "../Users/LED/LED.h"
#include "controller.h"
#include "../Users/buzzer/buzzer.h"

//（双上11为红⾊，双中33为⽩⾊，左中右上31为绿⾊，左上右中13为蓝⾊）
void UART_Controller(uint8_t* buf){
    //HAL_UART_Transmit_DMA(&huart1,buf,18);//可选发送接收到的数据
    uint8_t left_buf,right_buf;
    //存放接收到的数据
    //左右摇杆 1上 3中 2下
    left_buf=((buf[5] >> 4) & 0x000C) >> 2;
    right_buf= ((buf[5] >> 4) & 0x0003);
    /* 处理数据 */
    pullrod_control_LED(left_buf,right_buf);
    pullrod_sing_a_song(left_buf,right_buf);
}

//摇杆控制LED灯
void pullrod_control_LED(uint8_t left_buf,uint8_t right_buf){
    switch(left_buf){//左摇杆控制颜色
        case 1://向上 红灯亮
            LED_Color_Set('R');
            break;
        case 3://中间 蓝灯亮
            LED_Color_Set('B');
            break;
        case 2://向下 绿灯亮
            LED_Color_Set('G');
            break;
    }
    switch(right_buf){//右摇杆控制频率
        case 1://向上 100ms
            LED_freq_set(100);
            break;
        case 3://中间 200ms
            LED_freq_set(200);
            break;
        case 2://向下 300ms
            LED_freq_set(300);
            break;
    }
}

//摇杆控制歌声
void pullrod_sing_a_song(uint8_t left_buf,uint8_t right_buf){
    static uint8_t sound[4][4],sound_init_flag=0;
    if(!sound_init_flag){
        sound_init_flag=1;
        sound[1][1]=1;//do
        sound[1][3]=2;//re
        sound[1][2]=3;//mi
        sound[3][1]=4;//fa
        sound[3][3]=5;//so
        sound[3][2]=6;//la
        sound[2][1]=7;//si
        sound[2][2]=8;//歌1
        sound[2][3]=9;//歌2
    }
    buzzer_scale_set(sound[left_buf][right_buf]);
}