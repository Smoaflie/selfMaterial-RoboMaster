#include "gyro.h"
extern float relativeRadian;//相对偏移弧度
fp32 gyro[3],accel[3];
//陀螺仪初始化
void gyro_init(void){
    BMI088_Init();
    HAL_delay(3000);//延时三秒，确保陀螺仪稳定
    BMI088_read(gyro,accel);
}
//重新设置零点
void gyro_setCurRadian(void){
    while(gyro[2])  BMI088_read(gyro,accel);    //等待陀螺仪稳定
    relativeRadian=0;
}
//计算相对偏移弧度
void gyro_calRelativeRadian(void){
    
}