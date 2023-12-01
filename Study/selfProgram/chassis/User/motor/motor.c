/*
 * @Author: Smoaflie
 * @Date: 2023-11-08 19:23:19
 * @LastEditors: Smoaflie
 * @LastEditTime: 2023-11-10 22:20:47
 * @Description: 请填写简介
 */
#include "motor.h"
#include "usr_CAN.h"
#include "controller.h"
#include <math.h>

// 增/删电机数需修改 motor_config 与 motor_getID 函数内容
ElectricMotor motor[6];//0号motor存储车身旋转的pid参数

extern CAN_HandleTypeDef hcan1;

#define mecanum_W 0.2 //机器人左右麦轮距离
#define mecanum_H 0.2 //机器人前后麦轮距离

/* 电机初始化(PID初始化) */
void motor_config(){
    CAN_FIleter_init();
    motor[1].type=MotorSpeed;
    motor[2].type=MotorSpeed;
    motor[3].type=MotorSpeed;
    motor[4].type=MotorSpeed;
    motor[5].type=MotorRadian;
    PID_struct_init(&motor[0].pid,2*M_PI,0.01,0.005,00001,0);//车身旋转PID参数
    PID_struct_init(&motor[1].pid,10000,1000,8,0.1,0);
    PID_struct_init(&motor[2].pid,10000,1000,8,0.1,0);
    PID_struct_init(&motor[3].pid,10000,1000,8,0.1,0);
    PID_struct_init(&motor[4].pid,10000,1000,8,0.1,0);
    PID_struct_init(&motor[5].pid,30000,5000,100,2,0);
}

/* 获取电机ID对应的电机数据指针 */
ElectricMotor* motor_getID(uint32_t input_motor_id){
    return &motor[input_motor_id];
}
/* 接收电机数据 */
void motor_DataHandle(uint32_t input_motor_id,uint8_t* data){
    ElectricMotor* motor_id = motor_getID(input_motor_id);
    motor_data_analyze(motor_id,data);
    if(motor_id->type==MotorSpeed)  motor_control_current_set(motor_id);
    else if(motor_id->type==MotorRadian)    motor_control_voltage_set(motor_id);
    motor_can_send_control_current();
}

/* 分析电机数据 */
//@in：电调编号，对应数据
void motor_data_analyze(ElectricMotor* motor_id,uint8_t data[]){
    motor_id->rotor_mechanical_angle=((uint16_t)data[0]<<8)|data[1];
    motor_id->rotor_rotate_speed=((uint16_t)data[2]<<8)|data[3];
    motor_id->rotor_torque_current=((uint16_t)data[4]<<8)|data[5];
    motor_id->motor_Temperature=data[6];
}

/* 根据获取到的数据调整电机控制电流 使达到某一速度 */
void motor_control_current_set(ElectricMotor* motor_id){
    motor_id->motor_control_current=pid_calc(&motor_id->pid,motor_id->rotor_rotate_speed,motor_id->motor_target);
}
/* 根据获取到的数据调整电机控制电压 使保持某一角度 */
void motor_control_voltage_set(ElectricMotor* motor_id){
    static uint8_t turn_flag = 0;
    int16_t value=0,value_l=0,target;
    value=motor_id->rotor_mechanical_angle;
    target=motor_id->motor_target;
    if(value-target>8191/2){    
        turn_flag=1;    //顺时针越过0点
    }else
    if(target-value>8191/2){    
        turn_flag=2;    //逆时针越过0点
    }
    if(turn_flag==1&&value>8192/2){
        value-=8192;
    }else
    if(turn_flag==2&&value<8192/2){
        value+=8192;
    }
    motor_id->motor_control_current=pid_calc(&motor_id->pid,value,target);
    if(value==value_l)  turn_flag=0;
    value_l=motor_id->rotor_mechanical_angle;
}

/* CAN通讯 向电调发送控制电流/电压参数 */
void motor_can_send_control_current(void){
    CAN_DataSent(&hcan1,0x200,motor[1].motor_control_current,motor[2].motor_control_current,motor[3].motor_control_current,motor[4].motor_control_current);
    CAN_DataSent(&hcan1,0x1ff,motor[5].motor_control_current,0,0,0);
}
//直接设置控制电流/电压值-DEBUG
void motor_control_current_set_direct(uint32_t input_motor_id,uint16_t value){
    ElectricMotor* motor_id = motor_getID(input_motor_id);
    motor_id->motor_control_current=value;
    motor_can_send_control_current();
}

//设置电机转速 输入为转每分-DEBUG
void motor_rotate_speed_set(uint32_t input_motor_id,int16_t rotate_speed){
    ElectricMotor* motor_id = motor_getID(input_motor_id);
    motor_id->motor_target=rotate_speed;
}
//设置电机角度 输入为/°-DEBUG
void motor_rotate_radian_set(uint32_t input_motor_id,int16_t angle_t){
    ElectricMotor* motor_id = motor_getID(input_motor_id);
    if(angle_t==0)    return;
    
    while(angle_t>=0&&angle_t<=360){    //限定范围
        if(angle_t>360) angle_t-=360;
        if(angle_t<0)   angle_t+=360;
    }

    int16_t radian = angle_t /360.0 * 8191;
    motor_id->motor_target=radian;
}

/************************************************************
 * @brief 控制麦克纳姆轮组底盘移动
 * 
 * @param speed 前进速度
 * @param radian 前进方向-弧度（以Y轴正半轴为起始轴，逆时针增大）
 * @param vo 机器人绕O点旋转速度，顺时针为正 rad/s
************************************************************/
void Mecanum_GO(float speed,float radian,float vo,float deviation){
    float vx = speed * cos(radian-deviation);  
    float vy = speed * sin(radian-deviation);
    int16_t v1,v2,v3,v4;   //对应四个电机
    /* 电机顺序（电机前转时：A朝右上，B朝左上）
       
       4A---3B

       1B---2A
       
    */
   
   v4 = vx-vy+vo*(mecanum_W/2+mecanum_H/2);
   v3 = vx+vy-vo*(mecanum_W/2+mecanum_H/2);
   v2 = vx-vy-vo*(mecanum_W/2+mecanum_H/2);
   v1 = vx+vy+vo*(mecanum_W/2+mecanum_H/2);

   motor_rotate_speed_set(1,v1);
   motor_rotate_speed_set(2,-v2);
   motor_rotate_speed_set(3,-v3);
   motor_rotate_speed_set(4,v4);
}

//控制云台旋转 通过PID控制云台转动到某个特定弧度
void PTZ_turn(float radian){
    motor_rotate_radian_set(5,radian);
}
/************************************************************
 * @brief 控制底盘移动 能通过PID控制底盘旋转到某个特定弧度移动
 * 
 * @param speed 速度
 * @param radian 移动弧度
 * @param car_radian 车身目标弧度
************************************************************/
void chasis_run(float speed,float radian,float deviation,float car_radian){
    float vo=0;//旋转速度

    static uint8_t turn_flag = 0;
    float value=0,value_l=0,target;
    value=motor[5].rotor_mechanical_angle;
    target=car_radian;
    if(value-target>M_PI*2){    
        turn_flag=1;    //顺时针越过0点
    }else
    if(target-value>M_PI*2){    
        turn_flag=2;    //逆时针越过0点
    }
    if(turn_flag==1&&value > M_PI){
        value-=M_PI*2;
    }else
    if(turn_flag==2&&value < M_PI){
        value+=M_PI*2;
    }
    vo=pid_calc(&motor[0].pid,value,target)*5000;
    value_l=value;
    if(value==value_l)  turn_flag=0;

    Mecanum_GO(speed,radian,-vo,deviation);
}