#include "gimbal.h"

#include "motor.h"
#include "controller.h"
#include "gyro.h"

float PTZangle_S = 0;    //确定云台相对底盘位置（主轴）
_gimbalFLAG gimbalFLAG;

/* 控制器控制云台移动 */
void gimbal_move_by_controller(void){
    gimbalFLAG = gimbal_HOLD;
    if (RC_GetData(RC_ROCKER_R) > 1) {
        PTZ_turn_bySpeed(5, -((int16_t)RC_CtrlData.rc.ch0 - 1024) * 4 / 660.0);
        gyro_setYaw();
    }
    if (RC_GetData(RC_ROCKER_R) > 1) {
        PTZ_turn_bySpeed(9, ((int16_t)RC_CtrlData.rc.ch1 - 1024) * 4 / 660.0);
        gyro_setPitch();
    }
}

/* 云台无力（关闭） */
void gimbal_down(void){
    gimbalFLAG = gimbal_OFF;
}

/* 设置车辆行进时云台位置（相对于底盘） */
void gimbal_location_set(void){
    gyro_setYaw();
    gyro_setPitch();
    PTZ_mainSet();
}

//控制云台旋转 通过PID控制云台转动到某个特定弧度角度
void PTZ_turn_byAngle(uint32_t motor_id,float angle){
    motor_targetSpeedSet_ByANGLE(motor_id,angle);
}
void PTZ_turn_bySpeed(uint32_t motor_id,float speed){
    motor_rotate_speed_set(motor_id,speed);
}

//设置当前位置为云台主轴
void PTZ_mainSet(void){
    PTZangle_S = gimbal_yaw_motor.rotor_mechanical_angle;
}

//获取云台当前轴信息(8191制)
int16_t PTZ_Get(void){
    int16_t value = gimbal_yaw_motor.rotor_mechanical_angle-PTZangle_S;
    return value;
}
//获取云台当前轴信息 (弧度)
float PTZ_RadianGet(void){
    float radian;
    radian = (gimbal_yaw_motor.rotor_mechanical_angle - PTZangle_S)/8191.0*M_PI*2;
    if(radian<0) radian+=M_PI*2;
    return radian;
}

//获取云台主轴信息 (8191制)
float PTZ_mainRadianGet(void){
    return PTZangle_S;
}


























