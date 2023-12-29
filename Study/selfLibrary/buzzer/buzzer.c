#include "buzzer.h"
#include "song.h"

#define Tclk 84000000
#define psc (1000-1)
//标准音阶 0 1-do-261.6 2-re-293.7以此类推
static uint16_t standard_scale_freq[]={0,262,294,330,350,393,441,495};
static uint16_t buzzer_scale_freq[8]={0};
//设置频率
void buzzer_freq_set(uint8_t freq_number){
    //时钟频率：84000000HZ
    //预分配arr:x:通过调节预分频值控制发声频率
    //重载值psc:1000-1
    //Tout= ((arr+1)*(psc+1))/Tclk (s)
    //freq=1/Tout
    //PWM占空比保持50%
    static uint8_t get_buzzer_freq_flag=1;
    if(get_buzzer_freq_flag){
        for(int i = 1 ; i <= 7 ; i++){
            buzzer_scale_freq[i]=Tclk/(psc+1)/standard_scale_freq[i]-1;
        }
        get_buzzer_freq_flag=0;
    }
    uint32_t arr = freq_number;
    if(arr < 10)
        {arr=buzzer_scale_freq[freq_number];}

    HAL_TIM_Base_Stop(&htim4);
    __HAL_TIM_SET_PRESCALER(&htim4, arr);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_Base_Start(&htim4);
}

/* 响n下 */
void buzzer_play(uint16_t n){
    while(n--){
        HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
        HAL_Delay(100);
        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
        HAL_Delay(100);
    }
}

/* 唱歌 */
void buzzer_sone(void){
    static uint32_t length;

    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);

    //起风了
    length = sizeof(wind_rise)/sizeof(wind_rise[0]);
    for(int i=0;i<(length/2);i++)
    {
        buzzer_freq_set(wind_rise[i*2]);
        HAL_Delay(wind_rise[i*2+1]*5);
    }

    HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
}