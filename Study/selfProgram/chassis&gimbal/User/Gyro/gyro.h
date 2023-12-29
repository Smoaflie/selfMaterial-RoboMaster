#ifndef __GYRO_H_
#define __GYRO_H_

#include "usr_main.h"
#include "bmi088_driver.h"

void gyro_init(void);

void gyro_calData(void);
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3]);
float kalmanFilter(float measurement,float* P,float* x_estimated);

void gyro_setYaw(void);
void gyro_setPitch(void);

float gyro_getYAW(void);
float gyro_getPitch(void);

float gyro_getRowGyro(void);
float gyro_getPitchGyro(void);
float gyro_getYawGyro(void);

void gyro_task(void);

#endif // !__GYRO_H_
