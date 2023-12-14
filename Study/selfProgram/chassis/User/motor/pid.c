#include "pid.h"
#include <math.h>

#define ABS(x) ((x > 0) ? (x) : (-x))

//限幅
void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

/*参数初始化*/
static void pid_param_init(struct __pid_t *pid, // PID参数初始化
                           float maxOutput,
                           float integralLimit,
                           float deadband,
                           float p,
                           float i,
                           float d)
{

    pid->IntegralLimit = integralLimit;
    pid->MaxOutput = maxOutput;
    pid->deadband = deadband;

    pid->p = p;
    pid->i = i;
    pid->d = d;
}

/*pid总体初始化*/
void PID_struct_init(
    pid_t *pid,
    uint32_t maxout,
    uint32_t intergral_limit,
    float deadband,

    float kp,
    float ki,
    float kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;

    /*init pid param */
    pid->f_param_init(pid, maxout, intergral_limit, deadband, kp, ki, kd);
}

/************************************************************
 * @brief pid计算
 * 
 * @param pid 
 * @param get 当前值
 * @param set 目标值
 * @return float 输出值
************************************************************/
float pid_calc(pid_t *pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get; // set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
        {return 0;}
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        {pid->iout=0;return 0;} 

    pid->pout = pid->p * pid->err[NOW];
    // pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
    pid->iout += pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);
    abs_limit(&(pid->iout), pid->IntegralLimit);

    pid->delta_u = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->delta_u), pid->MaxOutput);
    // pid->delta_out = pid->last_delta_out + pid->delta_u;
    // abs_limit(&(pid->delta_out), pid->MaxOutput);

    // pid->last_delta_out = pid->delta_out; // update last time

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];

    // return pid->delta_out;
    return pid->delta_u;
}

/************************************************************
 * @brief 对周期变化的数据通过PID计算获取值，带过零点处理
 * 
 * @param pid 
 * @param get 当前值
 * @param set 目标值
 * @param limit 周期
 * 
 * @return float 输出值
************************************************************/
float pid_calc_circle(pid_t *pid, float get, float set, float limit){
    
    static uint8_t turn_flag = 0;
    float value=0,value_l=0,target;
    value=get;
    target=set;
    if(value-target>limit/2){    
        turn_flag=1;    //顺时针越过0点
    }else
    if(target-value>limit/2){    
        turn_flag=2;    //逆时针越过0点
    }
    if(turn_flag==1&&value > limit/2){
        value-=limit;
    }else
    if(turn_flag==2&&value < limit/2){
        value+=limit;
    }
    value_l=value;
    if(value==value_l)  turn_flag=0;

    return pid_calc(pid,value,target);    
}