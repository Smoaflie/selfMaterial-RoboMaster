#include "chassis.h"
#include "motor.h"
#include <math.h>
#include "gimbal.h"
#include "controller.h"

float chassis_speed,chassis_radian,chassis_vo; //底盘运动参数

/************************************************************
 * @brief 控制底盘移动
 * 
 * @param speed 前进速度
 * @param radian 前进方向-弧度（以Y轴正半轴为起始轴，逆时针增大）
 * @param vo 机器人绕O点旋转速度，顺时针为正 rad/s
 * @param deviation 指向轴相对于车的偏差值
************************************************************/
void chassis_speed_set(float speed,float radian,float vo,float deviation){
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

void chassis_off(void){
    chassis_speed=0;
    chassis_radian=0;
    chassis_vo = 0;
}
/************************************************************
 * @brief 控制底盘跟随云台移动

 * @param speed 速度
************************************************************/
void chassis_run_with_gimbal(uint16_t speed){
    chassis_speed=speed*RC_GetData(RC_ROCKER_R) / 100;
    chassis_radian=RC_CtrlData.Ctrl.ROCKER_R_DEG;
    chassis_vo = pid_calc_circle(&chassis_motor[0].pid,PTZ_Get(),0,8191);
}

/************************************************************
 * @brief 控制底盘小陀螺方式移动

 * @param f_speed 移动速度
 * @param t_speed 小陀螺速度
************************************************************/
void chassis_top(uint16_t f_speed,uint16_t t_speed){
    chassis_speed=f_speed*RC_GetData(RC_ROCKER_R) / 100;
    chassis_radian=RC_CtrlData.Ctrl.ROCKER_R_DEG;
    chassis_vo = t_speed;
}

/************************************************************
 * @brief CAN发送底盘电机控制电流
************************************************************/
void chassis_current_send(void){
    for(int i = 1 ; i <= 4 ; i++){
        ElectricMotor *p_motor=motor_getID(i);
        motor_control_current_set(p_motor);
    }
    CAN_DataSent(&hcan1,0x200,chassis_motor[1].motor_control_current,chassis_motor[2].motor_control_current,chassis_motor[3].motor_control_current,chassis_motor[4].motor_control_current);
}

/************************************************************
 * @brief 控制底盘移动
************************************************************/
void chasis_run(void){
    chassis_speed_set(chassis_speed,chassis_radian,chassis_vo,PTZ_RadianGet());
    chassis_current_send();
}


/* 底盘任务 放在freertos中 */
void chassis_task(void){
    chasis_run();
}

