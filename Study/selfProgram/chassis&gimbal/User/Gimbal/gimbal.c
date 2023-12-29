#include "gimbal.h"

#include "motor.h"
#include "controller.h"
#include "gyro.h"
#include <math.h>

#define gimbal_Yaw 5
#define gimbal_Pitch 9
float gimbalangle_S = 0;    //确定云台相对底盘位置（主轴）
_gimbalFLAG gimbalFLAG;
float t_speed = 0; // 测试PID用的参数

// 主轴机械角度由调试车辆时读取设置
uint16_t gimbal_MachineAngle_front = 1356;
uint16_t gimbal_MachineAngle_back = 5505; //云台指向底盘前/后面时的机械角度


/* 控制器控制云台移动 */
void gimbal_move_by_controller(void){
    gimbalFLAG = gimbal_HOLD;
    
    // float t=HAL_GetTick();
    // t_speed = 0.785*sin(1.884*t/1000)+1.305;
    // t_speed *=4;
    // gimbal_turn_setSpeed(gimbal_Yaw, t_speed);

    // 检测左0右1摇杆是否归零 跳变沿标志位（当归零时触发一次）,0为此前为0，1为此前大于0，2为上升沿，4为下降沿
    static uint8_t JumpEdge_flag[2];
    /* 跳变沿判断 */
    JumpEdge_flag[0]=JumpEdge_flag[0]<2?(((JumpEdge_flag[0]==1)^(RC_GetData(RC_ROCKER_L)>0))?0x02<<JumpEdge_flag[0]:JumpEdge_flag[0]):(RC_GetData(RC_ROCKER_L)>0?1:0);
    JumpEdge_flag[1]=JumpEdge_flag[1]<2?(((JumpEdge_flag[1]==1)^(RC_GetData(RC_ROCKER_R)>0))?0x02<<JumpEdge_flag[1]:JumpEdge_flag[1]):(RC_GetData(RC_ROCKER_R)>0?1:0);

    float ch2 = ((int16_t)RC_CtrlData.rc.ch2 - 1024)/660.0;
    float ch3 = ((int16_t)RC_CtrlData.rc.ch3 - 1024)/660.0;
    /* Yaw轴 */
    if(ch2 != 0)
        {gimbal_turn_setAngle(gimbal_Yaw, ch2 * 50);}    
    else if(JumpEdge_flag[0]==4)
        {gimbal_turn_setAngle(gimbal_Yaw,0);}


    /* Pitch轴 */
    if(ch3 != 0){
        /* 人工限位 机械角度手动获取设置 */
        if((gimbal_pitch_motor.rotor_mechanical_angle > 4455 && ch3 < 0) || (gimbal_pitch_motor.rotor_mechanical_angle < 3670 && ch3 > 0))
            gimbal_turn_setSpeed(gimbal_Pitch,0);
        else 
            gimbal_turn_setAngle(gimbal_Pitch, - ch3 * 5);
    }else if(JumpEdge_flag[0]==4)
        {gimbal_turn_setAngle(gimbal_Pitch,0);}
        

        // gimbal_turn_setSpeed(gimbal_Pitch, t_speed);
    
}

/* 云台无力（关闭） */
void gimbal_down(void){
    gimbalFLAG = gimbal_OFF;
}

/* 重新设置车辆行进时云台位置（相对于底盘） */
void gimbal_location_reset(void){
    gyro_setYaw();
    gyro_setPitch();
    gimbal_mainSet();
}

//控制云台旋转 通过PID控制云台转动到某个特定弧度角度
void gimbal_turn_byAngle(uint32_t motor_id,float angle){
    motor_targetSpeedSet_ByANGLE(motor_id,angle);
}

//设置云台角度
void gimbal_turn_setAngle(uint32_t motor_id,float angle){
    extern fp32 yaw_t,pitch_t;
    if(motor_id==gimbal_Yaw)
        yaw_t = angle;
    if(motor_id==gimbal_Pitch)
        pitch_t = angle;
}

//设置云台速度
void gimbal_turn_setSpeed(uint32_t motor_id,float speed){
    motor_rotate_speed_set(motor_id,speed);
}

//设置云台主轴
void gimbal_mainSet(void){
    //设置当前位置为云台主轴
    // gimbalangle_S = gimbal_yaw_motor.rotor_mechanical_angle;

    // 主轴机械角度由调试车辆时读取设置
    uint16_t a=gimbal_MachineAngle_front;//机械角度-正面
    uint16_t b=gimbal_MachineAngle_back;//机械角度-背面
    uint16_t angle = gimbal_yaw_motor.rotor_mechanical_angle;

    if( (angle > a-8191/4 && angle < a+8191/4) || (a>8191/4 ? 0 : (angle>8191-a)) || (a<8191*0.75 ? 0 : (angle<a+8191/4-8191)))
        gimbalangle_S=a;
    else gimbalangle_S=b;
}

//获取云台当前轴信息(8191制)
int16_t gimbal_Get(void){
    int16_t value = gimbal_yaw_motor.rotor_mechanical_angle-gimbalangle_S;
    return value;
}
//获取云台当前轴信息 (弧度)
float gimbal_RadianGet(void){
    float radian;
    radian = (gimbal_yaw_motor.rotor_mechanical_angle - gimbal_MachineAngle_front)/8191.0*M_PI*2;
    if(radian<0) radian+=M_PI*2;
    return radian;
}

//获取云台主轴信息 (8191制)
float gimbal_mainRadianGet(void){
    return gimbalangle_S;
}

/* 云台任务 放在freertos中 */
void gimbal_task(void){
    if(gimbalFLAG==gimbal_OFF)  return;

    if(gimbalFLAG==gimbal_HOLD){
        //保持模式，保持云台指向一定方向
        if(gimbal_yaw_motor.motor_target==0)
            {motor_targetSpeedSet_ByANGLE(gimbal_Yaw,0);}
    }
    if(gimbal_pitch_motor.motor_target==0)  motor_targetSpeedSet_ByANGLE(gimbal_Pitch,0);

    motor_control_current_set(&gimbal_yaw_motor);
    motor_control_current_set(&gimbal_pitch_motor);

    CAN_DataSent(&hcan1,0x2FF,gimbal_pitch_motor.motor_control_current,0,0,0);
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3);
    CAN_DataSent(&hcan1,0x1ff,gimbal_yaw_motor.motor_control_current,0,0,0);
}
