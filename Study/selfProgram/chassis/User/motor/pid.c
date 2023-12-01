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
                           float p,
                           float i,
                           float d)
{

    pid->IntegralLimit = integralLimit;
    pid->MaxOutput = maxOutput;

    pid->p = p;
    pid->i = i;
    pid->d = d;
}

/*pid总体初始化*/
void PID_struct_init(
    pid_t *pid,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;

    /*init pid param */
    pid->f_param_init(pid, maxout, intergral_limit, kp, ki, kd);
}

/************************************************************
 * @brief pid计算
 * 
 * @param pid 
 * @param get 当前值
 * @param set 目标值
 * @return int16_t 输出值
************************************************************/
int16_t pid_calc(pid_t *pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get; // set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
    pid->iout = pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);
    abs_limit(&(pid->iout), pid->IntegralLimit);

    pid->delta_u = pid->pout + pid->iout + pid->dout;
    pid->delta_out = pid->last_delta_out + pid->delta_u;
    abs_limit(&(pid->delta_out), pid->MaxOutput);

    pid->last_delta_out = pid->delta_out; // update last time

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];

    return (int16_t)pid->delta_out;
}