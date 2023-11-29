#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "pid.h"
typedef struct __ElectricMotor{
    
    uint16_t rotor_mechanical_angle;//转子机械角度：0~8191（对应0~360°）
    int16_t rotor_rotate_speed;//转子转速
    int16_t rotor_torque_current;//转子转矩电流
    uint8_t motor_Temperature;//电机温度

    int16_t motor_control_current;//控制电流

    int16_t motor_target_rotate_speed;//目标转速

    pid_t pid;  //pid控制参数
}ElectricMotor;

void motor_config();
void motor_DataHandle(uint32_t input_motor_id,uint8_t* data);
void motor_data_analyze(ElectricMotor* motor_id,uint8_t* data);
void motor_control_current_set(ElectricMotor* motor_id);
void motor_can_send_control_current(void);
void motor_rotate_speed_set(uint32_t input_motor_id,uint16_t rotate_speed);
void motor_control_current_set_direct(uint32_t input_motor_id,uint16_t value);

#endif //__MOTOR_H
