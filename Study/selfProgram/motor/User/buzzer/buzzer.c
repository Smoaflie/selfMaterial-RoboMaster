#include "buzzer.h"
#include "main.h"
#include "tim.h"
#include "song.h"
#include "../Users/Delay/delay.h"

#define Tclk 84000000
#define psc (1000-1)
//标准音阶 0 1-do-261.6 2-re-293.7以此类推
static uint32_t standard_scale_freq[]={0,262,294,330,350,393,441,495};
static uint32_t buzzer_scale_freq[8];
uint8_t sing_song_flag,sing_song_state=0;
//唱音阶
void buzzer_scale_set(uint8_t state){
    if(state==8||state==9)  sing_song_flag=1;
    else    sing_song_flag=0;
    sing_song_state=state;
}
//设置频率
void buzzer_freq_set(uint8_t freq_number){
    //时钟频率：84000000HZ
    //预分配arr:x:通过调节预分频值控制发声频率
    //重载值psc:1000-1
    //Tout= ((arr+1)*(psc+1))/Tclk (s)
    //freq=1/Tout
    //为提高精度，多除一位小数
    static uint8_t get_buzzer_freq_flag=1;
    if(get_buzzer_freq_flag){
        for(int i = 1 ; i <= 7 ; i++){
            buzzer_scale_freq[i]=Tclk/(psc+1)/standard_scale_freq[i]-1;
        }
        get_buzzer_freq_flag=0;
    }
    __HAL_TIM_SET_PRESCALER(&htim4,buzzer_scale_freq[freq_number]);
}

//唱歌
void buzzer_sing_song(void){
    static uint32_t length;
    sing_song_flag=1;
    switch(sing_song_state){
        case 0:
            buzzer_freq_set(0);
            return;
        case 8://孤勇者
            length = sizeof(solitary_brave)/sizeof(solitary_brave[0]);
            for(uint8_t i=0;i<(length/2)&&sing_song_flag;i++){
                __HAL_TIM_SET_PRESCALER(&htim4,solitary_brave[i*2]);
                // HAL_Delay(5*solitary_brave[i*2+1]);
                //经测试delay_1ms和HAL_Delay等效
                delay_1ms(5*solitary_brave[i*2+1]);
            }
            break;
        case 9:
            length = sizeof(wind_rise)/sizeof(wind_rise[0]);
            for(int i=0;i<(length/2)&&sing_song_flag;i++)
            {
                __HAL_TIM_SET_PRESCALER(&htim4,wind_rise[i*2]);
                delay_1ms(wind_rise[i*2+1]*5);
            }
            break;
        default:
            buzzer_freq_set(sing_song_state);
            delay_10us(30000);
            buzzer_freq_set(0);
            delay_1ms(2000);
    }
}
