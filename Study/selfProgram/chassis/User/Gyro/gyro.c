#include "gyro.h"
fp32 gyro[3],accel[3];//pitch raw yaw 长宽高
fp32 yaw = 0;
uint32_t d_t,l_t;
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
    while(gyro[2]){while(gyro[2]){BMI088_read(gyro,accel);}BMI088_read(gyro,accel);}    //等待陀螺仪稳定
    yaw = 0;
    l_t=HAL_GetTick();
}
//计算绝对数据
void gyro_calData(void){
    BMI088_read(gyro,accel);
    uint32_t n_t = HAL_GetTick();
    d_t=n_t - l_t;
    l_t=n_t;
	/* 通过积分计算角度 */
	yaw += gyro[2] * d_t;		/* 积分结算Yaw轴角度 */

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
//获取yaw绝对弧度
float gyro_getYAW(void){
    return yaw/180.0*M_PI;
}