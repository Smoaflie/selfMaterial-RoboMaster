/*
 * @Author: Smoaflie
 * @Date: 2023-11-08 19:23:19
 * @LastEditors: Smoaflie
 * @LastEditTime: 2023-11-10 22:20:47
 * @Description: 
 */
#include "motor.h"
#include "usr_CAN.h"
#include "gyro.h"
#include "gimbal.h"
#include "chassis.h"

ElectricMotor chassis_motor[5];//0存储车身旋转(参考值为云台电机机械角度）的pid参数,1-4为轮组电机
ElectricMotor gimbal_yaw_motor,gimbal_pitch_motor;

extern CAN_HandleTypeDef hcan1;

pid_t YawPID,PitchPID;//角度环

/* 电机初始化(PID初始化) */
void motor_config(){
    CAN_FIleter_init();
    chassis_motor[1].type=MotorChassis;
    chassis_motor[2].type=MotorChassis;
    chassis_motor[3].type=MotorChassis;
    chassis_motor[4].type=MotorChassis;
    gimbal_yaw_motor.type=MotorGimbal;
    gimbal_pitch_motor.type=MotorGimbal;
    PID_struct_init(&chassis_motor[0].pid,3000,500,100,1,0,0);//车身旋转PID参数(参考值为云台电机机械角度）
    PID_struct_init(&chassis_motor[1].pid,10000,1000,0,8,0.1,0);
    PID_struct_init(&chassis_motor[2].pid,10000,1000,0,8,0.1,0);
    PID_struct_init(&chassis_motor[3].pid,10000,1000,0,8,0.1,0);
    PID_struct_init(&chassis_motor[4].pid,10000,1000,0,8,0.1,0);

    PID_struct_init(&gimbal_yaw_motor.pid,30000,30000,0,8000,100,1000);
    PID_struct_init(&gimbal_pitch_motor.pid,30000,30000,0,10000,10,1000);

    PID_struct_init(&YawPID,4,4,0,0.0001,0,0);//角度环  设置±1°死区
    PID_struct_init(&PitchPID,4,4,0,0.5,0,0);//角度环
}

/* 获取电机ID对应的电机数据指针 */
ElectricMotor* motor_getID(uint32_t input_motor_id){
    if(input_motor_id < 5)  return &chassis_motor[input_motor_id];
    else if(input_motor_id == 5)    return &gimbal_yaw_motor;
    else if(input_motor_id == 9)    return &gimbal_pitch_motor;
    else return NULL;
}

/* 接收电机数据 */
void motor_dataHandle(uint32_t input_motor_id,uint8_t* data){
    ElectricMotor* p_motor = motor_getID(input_motor_id);
    motor_data_analyze(p_motor,data);
}

/* 分析电机数据 */
void motor_data_analyze(ElectricMotor* p_motor,uint8_t data[]){
    p_motor->rotor_mechanical_angle=((uint16_t)data[0]<<8)|data[1];
    p_motor->rotor_rotate_speed=((uint16_t)data[2]<<8)|data[3];
    p_motor->rotor_torque_current=((uint16_t)data[4]<<8)|data[5];
    p_motor->motor_Temperature=data[6];
}

/* PID速度环 设置电机控制电流 */
void motor_control_current_set(ElectricMotor* p_motor){
    // /* 急停 */
    // if(p_motor->stop_flag==1){
    //     p_motor->motor_control_current= - p_motor->rotor_torque_current;
    //     p_motor->stop_flag=0;
    //     return;
    // }

    if(p_motor->type==MotorChassis){
        p_motor->motor_control_current=(int16_t)pid_calc(&p_motor->pid,p_motor->rotor_rotate_speed,p_motor->motor_target);
    }
    if(p_motor->type==MotorGimbal){
        gyro_calData();
        if(p_motor==&gimbal_yaw_motor) 
            p_motor->motor_control_current = (int16_t)pid_calc(&p_motor->pid,- gyro_getYawGyro(),p_motor->motor_target);
        else if(p_motor==&gimbal_pitch_motor)    
            p_motor->motor_control_current = -(int16_t)pid_calc(&p_motor->pid,gyro_getPitchGyro(),p_motor->motor_target);
    }
    p_motor->motor_target=0;
    return;
    
}

/* 直接设置电机转速 */
void motor_rotate_speed_set(uint32_t input_motor_id,float rotate_speed){
    ElectricMotor* p_motor = motor_getID(input_motor_id);
    p_motor->motor_target=rotate_speed;
}
/* PID角度环 设置电机目标转速 *///数据由陀螺仪提供
void motor_targetSpeedSet_ByANGLE(uint32_t input_motor_id,float angle_t){
    ElectricMotor* p_motor = motor_getID(input_motor_id);
    while(!(angle_t>=0&&angle_t<=360)){    //限定范围
        if(angle_t>360) angle_t-=360;
        if(angle_t<0)   angle_t+=360;
    }
    float vo;   //电机速度

    /* 角度环，得到目标速度 */
    if(p_motor==&gimbal_yaw_motor)
        vo = pid_calc_circle(&YawPID,gyro_getYAW(),angle_t,360);   
    else if(p_motor==&gimbal_pitch_motor)
        vo = pid_calc_circle(&PitchPID,gyro_getPitch(),angle_t,360);   
    else    vo=0;
    p_motor->motor_target=vo;
}

/* CAN通讯 向电调发送控制电流/电压参数 */
void motor_can_send_control_current(void){
    CAN_DataSent(&hcan1,0x200,chassis_motor[1].motor_control_current,chassis_motor[2].motor_control_current,chassis_motor[3].motor_control_current,chassis_motor[4].motor_control_current);
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3);
    if(!(gimbalFLAG==gimbal_OFF)){
        CAN_DataSent(&hcan1,0x2FF,gimbal_pitch_motor.motor_control_current,0,0,0);
        CAN_DataSent(&hcan1,0x1ff,gimbal_yaw_motor.motor_control_current,0,0,0);
    }
}

//计算各电机所需电流值并发送(使车辆按指令行进)
void car_run(void){
    // gyro_calData();
    for(int i=1;i<10;i++){
        ElectricMotor *p_motor=motor_getID(i);
        if(p_motor == NULL) continue;

        if(gimbalFLAG==gimbal_HOLD){
          //保持模式，保持云台指向一定方向
          if(p_motor==&gimbal_yaw_motor && p_motor->motor_target==0)
            {motor_targetSpeedSet_ByANGLE(5,0);}
        }
        if(p_motor==&gimbal_pitch_motor && p_motor->motor_target==0)  motor_targetSpeedSet_ByANGLE(9,0);
        motor_control_current_set(p_motor);
    }
    motor_can_send_control_current();
}