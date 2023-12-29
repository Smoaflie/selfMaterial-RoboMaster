#include "usr_main.h"

#include "LED.h"
#include "buzzer.h"
#include "motor.h"
#include "usr_CAN.h"

int usr_main(void){
    CAN_FIleter_init();
    motor_config();
    

    // ! NOTE:while死循环不能删除
    while(1){
        if(!HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
        {   
            LED_Color_Set('G'); 
            motor_rotate_speed_set(2,2000);
            extern ElectricMotor motor2;
            motor_control_current_set(&motor2);
            motor_can_send_control_current();}
        else {motor_rotate_speed_set(2,0);LED_Color_Set('R');}
        HAL_Delay(1);
    }
    
}
