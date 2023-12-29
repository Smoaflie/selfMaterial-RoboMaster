#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#include "usr_main.h"

void chassis_off(void);
void chassis_run_with_gimbal(uint16_t speed);
void chassis_top(uint16_t f_speed,uint16_t t_speed);

void chassis_task(void);

#endif // !__CHASSIS_H_