#include "gimbal.h"

#include "motor.h"
#include "controller.h"
#include "gyro.h"

#define gimbal_Yaw 5
#define gimbal_Pitch 9
float PTZangle_S = 0;    //确定云台相对底盘位置（主轴）
_gimbalFLAG gimbalFLAG;
float t_speed = 0; // 测试PID用的参数

/* 控制器控制云台移动 */
void gimbal_move_by_controller(void){
    gimbalFLAG = gimbal_HOLD;
    if (RC_GetData(RC_ROCKER_L) > 1) {
        PTZ_turn_setSpeed(gimbal_Yaw, -((int16_t)RC_CtrlData.rc.ch2 - 1024) * 4 / 660.0);
        gyro_setYaw();
    }
    if (RC_GetData(RC_ROCKER_L) > 1) {
        /* 人工限位 机械角度手动获取设置 */
        if(gimbal_pitch_motor.rotor_mechanical_angle > 4755 || gimbal_pitch_motor.rotor_mechanical_angle < 3570)
            PTZ_turn_setSpeed(gimbal_Pitch,0);
        else 
            PTZ_turn_setSpeed(gimbal_Pitch, ((int16_t)RC_CtrlData.rc.ch3 - 1024) * 2 / 660.0);
        gyro_setPitch();

        
    }
    // PTZ_turn_setSpeed(gimbal_Pitch, t_speed);
}

/* 云台无力（关闭） */
void gimbal_down(void){
    gimbalFLAG = gimbal_OFF;
}

/* 重新设置车辆行进时云台位置（相对于底盘） */
void gimbal_location_reset(void){
    gyro_setYaw();
    gyro_setPitch();
    PTZ_mainSet();
}

//控制云台旋转 通过PID控制云台转动到某个特定弧度角度
void PTZ_turn_byAngle(uint32_t motor_id,float angle){
    motor_targetSpeedSet_ByANGLE(motor_id,angle);
}

void PTZ_turn_setSpeed(uint32_t motor_id,float speed){
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

    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3);
    CAN_DataSent(&hcan1,0x2FF,gimbal_pitch_motor.motor_control_current,0,0,0);
    CAN_DataSent(&hcan1,0x1ff,gimbal_yaw_motor.motor_control_current,0,0,0);
}

void _GimbalTask(void *argument)
{
  /* USER CODE BEGIN _GimbalTask */
  /* Infinite loop */
  for(;;)
  {
    gimbal_task();
    
    vTaskSuspend(NULL);
    // osDelay(1);
  }
  /* USER CODE END _GimbalTask */
}






















