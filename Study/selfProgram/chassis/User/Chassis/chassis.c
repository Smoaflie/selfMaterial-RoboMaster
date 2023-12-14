#include "chassis.h"
#include "motor.h"
#include <math.h>
#include "gimbal.h"

/************************************************************
 * @brief 控制麦克纳姆轮组底盘移动
 * 
 * @param speed 前进速度
 * @param radian 前进方向-弧度（以Y轴正半轴为起始轴，逆时针增大）
 * @param vo 机器人绕O点旋转速度，顺时针为正 rad/s
 * @param deviation 指向轴相对于车的偏差值
************************************************************/
void Mecanum_GO(float speed,float radian,float vo,float deviation){
    float vx = speed * cos(radian+deviation);  
    float vy = speed * sin(radian+deviation);
    int16_t v1,v2,v3,v4;   //对应四个电机
    /* 电机顺序（电机前转时：A朝右上，B朝左上）
       
       4A---3B

       1B---2A
       
    */
   
   v4 = vx-vy+vo;
   v3 = vx+vy-vo;
   v2 = vx-vy-vo;
   v1 = vx+vy+vo;

   motor_rotate_speed_set(1,v1);
   motor_rotate_speed_set(2,-v2);
   motor_rotate_speed_set(3,-v3);
   motor_rotate_speed_set(4,v4);
}
/************************************************************
 * @brief 控制全向轮轮组底盘移动
 * 
 * @param speed 前进速度
 * @param radian 前进方向-弧度（以Y轴正半轴为起始轴，逆时针增大）
 * @param vo 机器人绕O点旋转速度，顺时针为正 rad/s
 * @param deviation 指向轴相对于车的偏差值
************************************************************/
void Omni_GO(float speed,float radian,float vo,float deviation){
    float vx = speed * cos(radian+deviation);  
    float vy = speed * sin(radian+deviation);
    int16_t v1,v2,v3,v4;   //对应四个电机
/*
    1A   2B
    4B   3A
*/
   v1 = vx-vy+vo;
   v2 = vx+vy-vo;
   v3 = vx-vy-vo;
   v4 = vx+vy+vo;
    motor_rotate_speed_set(1,-v1);
    motor_rotate_speed_set(2,v2);
    motor_rotate_speed_set(3,v3);
    motor_rotate_speed_set(4,-v4);
}

/************************************************************
 * @brief 控制底盘跟随云台移动

 * @param speed 速度
 * @param radian 移动弧度
************************************************************/
void chasis_run_with_gimbal(float speed,float radian){
    float v=0;//旋转速度
    v = pid_calc_circle(&chassis_motor[0].pid,PTZ_Get(),0,8191);

    chasis_run(speed,radian,v);
}

/************************************************************
 * @brief 控制底盘移动(不跟随云台，但前进方向仍以云台指向为参考)

 * @param speed 速度
 * @param radian 移动弧度
************************************************************/
void chasis_run(float speed,float radian,float vo){
    // Mecanum_GO(speed,radian,v,PTZ_RadianGet());
    Omni_GO(speed,radian,vo,PTZ_RadianGet());
}
