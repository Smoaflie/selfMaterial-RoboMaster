#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "usr_main.h"

typedef enum{
    gimbal_ON = 0,
    gimbal_OFF,
    gimbal_HOLD   //保持,受外力不移动
}_gimbalFLAG;   //云台状态标识

extern _gimbalFLAG gimbalFLAG;

void gimbal_location_reset(void);

void gimbal_down(void);

void gimbal_move_by_controller(void);

void gimbal_turn_byAngle(uint32_t motor_id,float angle);

void gimbal_turn_setSpeed(uint32_t motor_id,float speed);
void gimbal_turn_setAngle(uint32_t motor_id,float angle);

void gimbal_mainSet(void);
int16_t gimbal_Get(void);
float gimbal_RadianGet(void);
float gimbal_mainRadianGet(void);

void gimbal_task(void);

#endif // !__GIMBAL_H
