/*
 * @Author: Smoaflie
 * @Date: 2023-12-29
 * @LastEditors: Smoaflie
 * @LastEditTime: 2023-11-10 22:20:47
 * @Description: 
 */
#ifndef __BUZZER_H
#define __BUZZER_H
#include "usr_main.h"

extern uint8_t sing_song_flag;

void buzzer_freq_set(uint8_t freq_number);

void buzzer_play(uint16_t n);

void buzzer_sone(void);

#endif //__BUZZER_H