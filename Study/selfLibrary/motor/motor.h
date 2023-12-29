/*
 * @Author: Smoaflie
 * @Date: 2023-12-29
 * @LastEditors: Smoaflie
 * @LastEditTime: 2023-11-10 22:20:47
 * @Description: 
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "usr_main.h"

#include "pid.h"

typedef enum{
    MotorChassis = 0, //底盘电机
    MotorGimbal     //云台电机
}MotorType; //电机调节目的

typedef struct __ElectricMotor{
    MotorType type;
    uint16_t rotor_mechanical_angle;//转子机械角度：0~8191（对应0~360°）
    int16_t rotor_rotate_speed;//转子转速
    int16_t rotor_torque_current;//转子转矩电流
    uint8_t motor_Temperature;//电机温度

    int16_t motor_control_current;//控制电流

    float motor_target;//目标值

    pid_t pid;  //pid控制参数

    uint8_t stop_flag; //急停标志位
}ElectricMotor;

extern ElectricMotor chassis_motor[5];
extern ElectricMotor gimbal_yaw_motor,gimbal_pitch_motor;

void motor_config();
void motor_dataHandle(uint32_t input_motor_id,uint8_t* data);
void motor_data_analyze(ElectricMotor* motor_id,uint8_t data[]);

ElectricMotor* motor_getID(uint32_t input_motor_id);

void motor_rotate_speed_set(uint32_t input_motor_id,float rotate_speed);
void motor_targetSpeedSet_ByANGLE(uint32_t input_motor_id,float angle_t);

void motor_control_current_set(ElectricMotor* motor_id);

void motor_can_send_control_current(void);

#endif //__MOTOR_H
