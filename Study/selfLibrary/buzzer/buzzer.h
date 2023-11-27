#ifndef __BUZZER_H
#define __BUZZER_H
#include "main.h"
extern uint8_t sing_song_flag;
//唱音阶
void buzzer_scale_set(uint8_t state);
//设置频率
void buzzer_freq_set(uint8_t freq_number);
//唱歌
void buzzer_sing_song(void);
#endif //__BUZZER_H