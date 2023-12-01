#ifndef __GYRO_H_
#define __GYRO_H_

#include "usr_main.h"

#include "bmi088_driver.h"

void gyro_init(void);
void gyro_setCurRadian(void);
void gyro_calData(void);
float gyro_getYAW(void);
#endif // !__GYRO_H_
