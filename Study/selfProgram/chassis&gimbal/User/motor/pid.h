/************************************************************
 * @file pid.h
 * @author DJI
 * @brief PID计算
 * @version 1.0
 * @date 2023-11-30
 * @note    该部分代码来自DJI官方例程
 * 
 * @copyright GNU General Public License v3.0
************************************************************/
#ifndef __PID_H
#define __PID_H

#include "usr_main.h"

enum
{
    LLAST = 0,
    LAST = 1,
    NOW = 2,
};

typedef struct __pid_t
{
    float p;
    float i;
    float d;

    float set[3]; // 目标值,包含NOW， LAST， LLAST上上次
    float get[3]; // 测量值
    float err[3]; // 误差

    float pout; // p输出
    float iout; // i输出
    float dout; // d输出

    float pos_out;      // 本次位置式输出
    float last_pos_out; // 上次输出
    float delta_u;      // 本次增量值
    float delta_out;    // 本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;

    float max_err;
    float deadband; // 死区 err < deadband return

    float MaxOutput;     // 输出限幅
    float IntegralLimit; // 积分限幅
    
    void (*f_param_init)(struct __pid_t *pid, // PID参数初始化
                         float maxOutput,
                         float integralLimit,
                         float deadband,
                         float p,
                         float i,
                         float d);
} pid_t;

void PID_struct_init(
    pid_t *pid,
    float maxout,
    float intergral_limit,
    float deadband,

    float kp,
    float ki,
    float kd);

float pid_calc(pid_t *pid, float get, float set);
float pid_calc_circle(pid_t *pid, float get, float set, float limit);
#endif //__PID_H