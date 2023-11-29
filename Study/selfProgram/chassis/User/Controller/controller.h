/************************************************************
 * @file controller.h
 * @author Smoaflie
 * @brief 使用DBUF通信接收控制器DT7数据
 * @version 1.0
 * @date 2023-11-29
 * @note C板已通过硬件将DBUF转化为UART信号
 *       大部分代码来自于官方文档-《RoboMaster 机器人专用遥控器（接收机）用户手册》
 *       1.0版本由于未使用鼠标，未添加相关代码，待补充ing
 * @copyright GNU General Public License v3.0
************************************************************/
#ifndef __CONTROLLER_H
#define __CONTROLLER_H
#include "usr_main.h"

//遥控器数据解析 结构体
typedef struct __RC_DataTypeDef{
    //遥控器数据
    struct
    {
        uint16_t ch0;   //通道0
        uint16_t ch1;   //通道1
        uint16_t ch2;   //通道2
        uint16_t ch3;   //通道3
        uint8_t s1;     //S1
        uint8_t s2;     //S2
    }rc;
    //鼠标数据
    struct
    {
        int16_t x;      //鼠标X轴
        int16_t y;      //鼠标Y轴
        int16_t z;      //鼠标Z轴
        uint8_t press_l;    //鼠标左键
        uint8_t press_r;    //鼠标右键
    }mouse;
    //按键数据
    struct
    {
        uint16_t v;     //按键
    }key;

    //处理后数据
    struct
    {
        double ROCKER_L;   //左摇杆推度
        double ROCKER_L_DEG;//左摇杆角度
        double ROCKER_R;//右摇杆推度
        double ROCKER_R_DEG;//右摇杆角度
    }Ctrl;
}RC_DataTypeDef;

//数据类型定义，角度(推度)使用整型(百分比)表示
typedef enum{
    RC_ROCKER_L = 0,    //左摇杆推度
    RC_ROCKER_L_DEG,    //左摇杆角度
    RC_ROCKER_R,        //右摇杆推度
    RC_ROCKER_R_DEG,    //右摇杆角度
    RC_PULLROD,         //拨杆状态
    RC_KEY              //按键
}RC_DataType;

extern uint8_t RC_RxBuffer[18];

//接收处理控制器数据-放在接收中断中
void RC_RecevieAnalysis(uint8_t *pData);

//供其他函数获取控制器数据
uint16_t RC_GetData(RC_DataType type);

#endif //!__CONTROLLER_H