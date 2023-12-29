#ifndef __LED_H
#define __LED_H
#include "usr_main.h"

void LED_Color_Set(uint8_t Color);

void LED_On(void);
void LED_Off(void);

void LED_lumx_set(float lumx);
void LED_freq_set(uint32_t freq);
void LED_lumx_control(void);
void LED_freq_control(void);

#endif//__LED_H
