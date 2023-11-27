/*
 * @Author: Smoaflie
 * @Date: 2023-11-08 19:23:19
 * @LastEditors: Smoaflie
 * @LastEditTime: 2023-11-10 22:20:47
 * @Description: 请填写简介
 */
#include "motor.h"
#include "pid.h"

pid_t pid1;
ElectricMotor motor_1;
extern CAN_HandleTypeDef hcan1;

/* 电机初始化(PID初始化) */
void motor_config(){
    PID_struct_init(&pid1,1000,1000,1.5,0.1,0);
}

/* 接收电机数据 */
void motor_data_get(uint32_t input_motor_id,uint8_t* data){
    ElectricMotor* motor_id;
    switch(input_motor_id){
        case 0x001:
            motor_id=&motor_1;
            break;
    }
    motor_data_analyze(motor_id,data);
    motor_control_current_set(motor_id);
    motor_can_send_control_current();
    
}
/* 分析接收到的电机数据 */
//@in：电调编号，对应数据
void motor_data_analyze(ElectricMotor* motor_id,uint8_t data[]){
    motor_id->rotor_mechanical_angle=((uint16_t)data[0]<<8)|data[1];
    motor_id->rotor_rotate_speed=((uint16_t)data[2]<<8)|data[3];
    motor_id->rotor_torque_current=((uint16_t)data[4]<<8)|data[5];
    motor_id->motor_Temperature=data[6];
}
/* 根据获取到的数据调整电机控制电流 */
void motor_control_current_set(ElectricMotor* motor_id){

    // static int16_t error_value,last_error,P_out,I_out,D_out;
    // error_value=motor_id->motor_target_rotate_speed-motor_id->rotor_rotate_speed;
    // P_out=P*error_value;
    // I_out+=I*error_value;
    // D_out=-D*(error_value-last_error);
    // last_error=error_value;
    // if(I_out>800)  I_out=800;
    // else if(I_out<-800) I_out=-800;
    motor_id->motor_control_current=pid_calc(&pid1,motor_id->rotor_rotate_speed,motor_id->motor_target_rotate_speed);
    
    // motor_id->motor_control_current=100*(P_out+D_out);
}
//直接设置控制电流值
void motor_control_current_set_direct(uint32_t input_motor_id,uint16_t value){
    ElectricMotor* motor_id;
    switch(input_motor_id){
        case 0x001:
            motor_id=&motor_1;
            break;
    }
    motor_id->motor_control_current=value;
    motor_can_send_control_current();
}
/* CAN方式向电调发送控制电流参数 */
void motor_can_send_control_current(void){
    CAN_TxHeaderTypeDef CAN1_Tx_Message;//定义发送数据结构体，用于存放IDE，RTR，DLC等内容
    uint8_t CAN1_Tx_Data[8];                   //用于暂存发送报文中的数据段
    CAN1_Tx_Message.DLC    = 8;                //数据长度为8
    CAN1_Tx_Message.IDE    = CAN_ID_STD;       //数据为标准格式
    CAN1_Tx_Message.RTR    = CAN_RTR_DATA;     //代表为数据帧
    CAN1_Tx_Message.StdId  = 0x200;
    CAN1_Tx_Data[0] = motor_1.motor_control_current >>8;
    CAN1_Tx_Data[1] = motor_1.motor_control_current;    
    CAN1_Tx_Data[2] = 0;
    CAN1_Tx_Data[3] = 0;
    CAN1_Tx_Data[4] = 0;
    CAN1_Tx_Data[5] = 0;
    CAN1_Tx_Data[6] = 0;
    CAN1_Tx_Data[7] = 0;
    CAN1_Tx_Data[8] = 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) //判断发送邮箱中是否存在空邮箱
    {
        HAL_CAN_AddTxMessage(&hcan1, &CAN1_Tx_Message, CAN1_Tx_Data, (uint32_t)CAN_TX_MAILBOX0);//将自定义报文添加到邮箱中
    }
}

/* 设置电机转速 输入为转每分 */
void motor_rotate_speed_set(uint32_t input_motor_id,uint16_t rotate_speed){
    ElectricMotor* motor_id;
    switch(input_motor_id){
        case 0x001:
            motor_id=&motor_1;
            break;
    }
    motor_id->motor_target_rotate_speed=rotate_speed;
}