#include "gyro.h"
fp32 gyro[3],accel[3];//pitch raw yaw 长宽高
fp32 gyro_offset[3];//计算零飘
fp32 yaw = 0,yaw_t,t=60;
uint32_t d_t,l_t;

// 定义卡尔曼滤波器的状态变量
float x_estimated = 0;  // 估计的状态值
float P = 0;           // 估计误差协方差


//陀螺仪初始化
void gyro_init(void){
    HAL_TIM_Base_Start_IT(&htim2);
    BMI088_Init();
    HAL_Delay(3000);//延时三秒，确保陀螺仪稳定
    gyro_setCurRadian();
    gyro_calData();
}
//重新设置零点
void gyro_setCurRadian(void){
    yaw = 0;
    yaw_t = 0;
    gyro_offset_calc(gyro_offset,gyro);
    l_t=HAL_GetTick();
}
//计算绝对数据
void gyro_calData(void){
    BMI088_read(gyro,accel);
    uint32_t n_t = HAL_GetTick();
    d_t=n_t - l_t;
    l_t=n_t;
	/* 通过积分计算角度 */
	yaw_t += (kalmanFilter(gyro[2])-gyro_offset[2]) * d_t/1000;		/* 积分结算Yaw轴角度 */
    yaw = yaw_t * t;
	/* 将角度限制在360° */
	if(yaw > 360.0f)
	{
		yaw -= 360;
	}
	else if(yaw < 0.0f)
	{
		yaw += 360;
	}
}
//获取yaw角度
float gyro_getYAW(void){
    return yaw;
}

// 卡尔曼滤波器函数
float kalmanFilter(float measurement) {
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

    uint16_t offset_time_count = 1000;
    while(offset_time_count--){
        BMI088_read(gyro,accel);
        gyro_offset[0] = gyro_offset[0] + 0.001f * gyro[0];
        gyro_offset[1] = gyro_offset[1] + 0.001f * gyro[1];
        gyro_offset[2] = gyro_offset[2] + 0.001f * gyro[2];
    }
}
