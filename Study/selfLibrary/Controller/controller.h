#ifndef __CONTROLLER_H
#define __CONTROLLER_H
#include "main.h"

/*
使⽤遥控器控制灯的颜⾊，初始时遥控器拨杆双下，此时灯不亮。通过遥控器拨杆控制灯的颜⾊
（双上11为红⾊，双中33为⽩⾊，左中右上31为绿⾊，左上右中13为蓝⾊）
*/
void UART_Controller(uint8_t* buf);
//摇杆控制LED灯
void pullrod_control_LED(uint8_t left_buf,uint8_t right_buf);
//摇杆控制歌声
void pullrod_sing_a_song(uint8_t left_buf,uint8_t right_buf);
#endif//__CONTROLLER_H
