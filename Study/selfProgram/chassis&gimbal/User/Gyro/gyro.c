#include "gyro.h"
#include <math.h>

fp32 gyro[3],accel[3];//pitch raw yaw 长宽高
fp32 gyro_offset[3];//计算零飘
fp32 yaw = 0,yaw_t,t=6349;
fp32 pitch = 0, pitch_t = 0;
uint32_t d_t,l_t;

// 定义卡尔曼滤波器的状态变量(_p为Pitch的滤波参数)
float x_estimated[3]={0.0, 0.0, 0.0};  // 估计的状态值
float P[3]={0.0, 0.0, 0.0};           // 估计误差协方差

typedef struct {
    double w, x, y, z;
} Quaternion;
Quaternion q = {1, 0, 0, 0}; // 初始四元数（单位四元数）

// 四元数乘法
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return result;
}

// 四元数归一化
Quaternion quaternion_normalize(Quaternion q) {
    double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    return q;
}

// 四元数计算
void quaternion_calc(void){
    double dt = d_t*0.001; // 时间间隔（秒）

    // 计算四元数更新
    double half_dt = 0.5 * dt;
    Quaternion delta_q;
    delta_q.w = 1;
    delta_q.x = kalmanFilter(gyro[0]-gyro_offset[0],&P[0],&x_estimated[0]) * half_dt;
    delta_q.y = kalmanFilter(gyro[1]-gyro_offset[1],&P[1],&x_estimated[1]) * half_dt;
    delta_q.z = kalmanFilter(gyro[2]-gyro_offset[2],&P[2],&x_estimated[2]) * half_dt;
    delta_q = quaternion_normalize(delta_q);
    q.w -= delta_q.x * q.x + delta_q.y * q.y + delta_q.z * q.z;
    q.x += delta_q.x * q.w + delta_q.y * q.z - delta_q.z * q.y;
    q.y -= delta_q.x * q.z + delta_q.y * q.w + delta_q.z * q.x;
    q.z += delta_q.x * q.y - delta_q.y * q.x + delta_q.z * q.w;
}

/* 陀螺仪初始化 */
void gyro_init(void){
    HAL_TIM_Base_Start_IT(&htim2);
    BMI088_Init();
    gyro_offset_calc(gyro_offset,gyro);
    gyro_setYaw();
    gyro_setPitch();
    gyro_calData();
}
/* 重新设置零点 */
void gyro_setYaw(void){
    yaw = 0;
    yaw_t = 0;
}
void gyro_setPitch(void){
    pitch=0;
    pitch_t=0;
}
/* 计算yaw,pitch角度 */
void gyro_calData(void){
    BMI088_read(gyro,accel);
    uint32_t n_t = HAL_GetTick();
    d_t=n_t - l_t;
    l_t=n_t;

    //四元数计算
    // quaternion_calc();

	/* 通过积分计算角度 */
	yaw_t -= (kalmanFilter(gyro[2]-gyro_offset[2],&P[2],&x_estimated[2]) * d_t/1000 * t);		/* 积分结算Yaw轴角度 */
    pitch_t += (kalmanFilter(gyro[1]-gyro_offset[1],&P[1],&x_estimated[1]) * d_t/1000 * t);		/* 积分结算Pitch轴角度 */
  
    yaw/=360;pitch/=360;
    yaw=yaw>0?yaw:-yaw;
    pitch=pitch>0?pitch:-pitch;
}

/* 获取yaw角度 */ 
float gyro_getYAW(void){
    yaw = yaw_t;
    /* 将角度限制在0-360° */
	while(yaw>360) yaw-=360;
    while(yaw<0)  yaw+=360; 

    return yaw;
}
/* 获取Pitch角度 */
float gyro_getPitch(void){
    pitch=pitch_t;
    /* 将角度限制在0-360° */
	while(pitch>360) pitch-=360;
    while(pitch<0)  pitch+=360; 

    return pitch;
}
/* 获取row角速度 */
float gyro_getRowGyro(void){
    return gyro[0]-gyro_offset[0];
}
/* 获取yaw角速度 */
float gyro_getYawGyro(void){
    return gyro[2]-gyro_offset[2];
}
/* 获取Pitch角速度 */
float gyro_getPitchGyro(void){
    return gyro[1]-gyro_offset[1];
}
/* 卡尔曼滤波器函数 */
float kalmanFilter(float measurement,float* P_t,float* x_estimated_t) {
    float P=*P_t ;
    float x_estimated = *x_estimated_t;
    // 预测步骤
    float Q = 0.1;  // 过程噪声协方差
    float R = 10;   // 测量噪声协方差

    float K;        // 卡尔曼增益

    // 更新状态估计
    x_estimated = x_estimated;  // 假设当前状态值为上一次的估计值

    // 更新估计误差协方差
    P = P + Q;

    // 更新卡尔曼增益
    K = P / (P + R);

    // 更新状态估计，根据测量值和卡尔曼增益
    x_estimated = x_estimated + K * (measurement - x_estimated);

    // 更新估计误差协方差
    P = (1 - K) * P;

    return x_estimated;
}

/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3])
{
    gyro_offset[0] = 0;
    gyro_offset[1] = 0;
    gyro_offset[2] = 0;

    uint16_t offset_time_count = 5000;
    while(offset_time_count--){
        BMI088_read(gyro,accel);
        HAL_Delay(1);
        gyro_offset[0] = gyro_offset[0] + 0.0001f * gyro[0];
        gyro_offset[1] = gyro_offset[1] + 0.0001f * gyro[1];
        gyro_offset[2] = gyro_offset[2] + 0.0001f * gyro[2];
    }
}

/* 陀螺仪任务 在freertos中 */
void gyro_task(void){
    gyro_calData();
}