#ifndef __USR_CAN_
#define __USR_CAN_

#include "usr_main.h"

void CAN_FIleter_init(void);
void CAN_DataSent(CAN_HandleTypeDef* hcan,uint16_t StdId,uint16_t data1,uint16_t data2,uint16_t data3,uint16_t data4);
#endif // !__USR_CAN_
