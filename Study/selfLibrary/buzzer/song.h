#include "main.h"
#define  proport          84000 	//84000（时钟频率）/1000(自动重载值)
                                             
#define  L1       ((proport/262)-1)//低调　do 的频率
#define  L2       ((proport/296)-1)//低调　re 的频率
#define  L3       ((proport/330)-1)//低调　mi 的频率
#define  L4       ((proport/349)-1)//低调　fa 的频率
#define  L5       ((proport/392)-1)//低调　sol 的频率
#define  L6       ((proport/440)-1)//低调　la 的频率
#define  L7       ((proport/494)-1)//低调　si 的频率
                                               
#define  M1       ((proport/523)-1)//中调　do 的频率
#define  M2       ((proport/587)-1)//中调　re 的频率
#define  M3       ((proport/659)-1)//中调　mi 的频率
#define  M4       ((proport/699)-1)//中调　fa 的频率
#define  M5       ((proport/784)-1)//中调　sol的频率
#define  M6       ((proport/880)-1)//中调　la 的频率
#define  M7       ((proport/988)-1)//中调　si 的频率
 
#define  H1       ((proport/1048)-1)//高调　do 的频率
#define  H2       ((proport/1176)-1)//高调　re 的频率
#define  H3       ((proport/1320)-1)//高调　mi 的频率
#define  H4       ((proport/1480)-1)//高调　fa 的频率
#define  H5       ((proport/1640)-1)//高调　sol的频率
#define  H6       ((proport/1760)-1)//高调　la 的频率
#define  H7       ((proport/1976)-1)//高调　si 的频率
#define  Z0       0//


uint16_t solitary_brave[]=
    {
        M6,50,M7,50,H1,50,H2,50,M7,50,H1,50,H1,100,Z0,10,    //爱你孤身走暗巷
        H1,50,M7,50,H1,50,H2,50,M7,50,H1,50,H1,100,Z0,10,     //爱你不跪的模样
        H1,50,H2,50,H3,50,H2,50,H3,50,H2,50,H3,100,H3,50,H3,50,H2,50,H3,100,H5,100,H3,100,Z0,10 //爱你对峙过绝望不肯哭一场

    };

uint16_t wind_rise[]=
    {    
        //前奏
        L7,25,M1,25,M2,25,M3,25,L3,50,M5,25,M3,25,M3,50,Z0,150, L7,25,M1,25,M2,25,M3,25,L2,50,M5,25,M3,25,M2,25,M3,25,M1,25,M2,25,L7,25,M1,25,L5,25,Z0,25, L7,25,M1,25,M2,25,M3,25,L3,50,M5,25,M3,75,Z0,150,L7,25,M1,25,M2,25,M3,25,L2,50,M5,25,M3,25,M2,25,M3,25,M1,25,M2,25,Z0,20,
        //这一路上走走停停 顺着少年漂流的痕迹
        M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,25,M3,25,M2,25,M1,25,L6,100,Z0,10,
        //迈出车站的前一刻 竟有些犹豫
        M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M3,25,M2,50,M1,50,M2,100,Z0,50,
        //不仅笑着这近乡情怯 仍无法避免
        M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M3,25,M2,50,M1,50,L6,100,Z0,10,
        //而长野的天 依旧那么暖 风吹起了从前
        M3,25,M2,25,M1,25,M2,25,M1,100, M3,25,M2,25,M1,25,M2,25,M1,50,M1,50, M5,25,M3,25,M2,25,M1,25,M2,25,M1,100,M1,150,Z0,30,
        //从前初识这世间 万般留恋 看着天边似在眼前 也甘愿赴汤蹈火去走它一遍
        M1,50,M2,50,M3,50,M1,50,M6,50,M5,25,M6,25,M6,50,M6,50, M1,25,M7,50,M6,25,M7,25,M7,100,Z0,5, M7,50,M6,25,M7,25,M7,50,M3,50,H1,25,H2,25,H1,25,M7,25,M6,50,M5,50, M6,50,M5,25,M6,25,M6,25,M5,25,M6,25,M5,25,M6,50,M5,25,M2,25,M2,25,M5,50,M5,50,M3,100,M3,100,Z0,25,
        //如今走过这世间 万般留恋 翻过岁月不同侧脸 措不及防闯入你的笑颜
        M1,50,M2,50,M3,50,M1,50,M6,50,M5,25,M6,25,M6,50,M6,50, M1,25,M7,50,M6,25,M7,25,M7,100,Z0,5, M7,50,M6,25,M7,25,M7,50,M3,50,H1,25,H2,25,H1,25,M7,25,M6,50,M5,50, M6,50,H3,25,H3,25,H3,50,M5,50,M6,50,H3,25,H3,25,H3,25,M5,50,M6,25,M6,100,M6,100,M6,100,Z0,25,
        //我曾难自拔于世界之大 也沉溺于其中梦话
        H1,50,H2,50,H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H2,25,H3,25,H3,50, H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,25,H3,50,H3,50,
        //不得真假 不做挣扎 不惧笑话
        H2,50,H1,25,M6,25,M6,25,H1,50, M6,25,H2,25,H1,25,M6,50,M6,25,H1,50,H1,50, H3,100,H3,25,H4,25,H3,50,H3,25,H2,50,H2,50,Z0,25,
        //我曾将青春翻涌成她 也曾指尖弹出盛夏 心之所动且就随缘去吧
        H1,50,H2,50,H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H2,50, H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H3,50,H3,50, H2,50,H1,25,M6,25,M6,25,H3,50,H3,50, H2,50,H1,25,M6,25,M6,25,H1,50,H1,50,H1,100,H1,100,Z0,10,
        //短短的路走走停停  也有了几分的距离
        M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,25,M3,25,M2,25,M1,25,L6,100,Z0,10,
        //不知抚摸的是故事还是段心情
        M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M3,25,M2,50,M1,50,M2,100,Z0,50,
        //也许期待的不过是与时间为敌
        M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M3,25,M2,50,M1,50,L6,100,Z0,10,
        //再次见到你 微凉晨光里 笑的很甜蜜
        M3,25,M2,25,M1,25,M2,25,M1,100, M3,25,M2,25,M1,25,M2,25,M1,50,M1,50, M5,25,M3,25,M2,25,M1,25,M2,25,M1,100,M1,150,Z0,30,
        //从前初识这世间 万般留恋 看着天边似在眼前 也甘愿赴汤蹈火去走它一遍
        M1,50,M2,50,M3,50,M1,50,M6,50,M5,25,M6,25,M6,50,M6,50, M1,25,M7,50,M6,25,M7,25,M7,100,Z0,5, M7,50,M6,25,M7,25,M7,50,M3,50,H1,25,H2,25,H1,25,M7,25,M6,50,M5,50, M6,50,M5,25,M6,25,M6,25,M5,25,M6,25,M5,25,M6,50,M5,25,M2,25,M2,25,M5,50,M5,50,M3,100,M3,100,Z0,25,
        //如今走过这世间 万般留恋 翻过岁月不同侧脸 措不及防闯入你的笑颜
        M1,50,M2,50,M3,50,M1,50,M6,50,M5,25,M6,25,M6,50,M6,50, M1,25,M7,50,M6,25,M7,25,M7,100,Z0,5, M7,50,M6,25,M7,25,M7,50,M3,50,H1,25,H2,25,H1,25,M7,25,M6,50,M5,50, M6,50,H3,25,H3,25,H3,50,M5,50,M6,50,H3,25,H3,25,H3,25,M5,50,M6,25,M6,100,M6,100,M6,100,Z0,25,
        //我曾难自拔于世界之大 也沉溺于其中梦话
        H1,50,H2,50,H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H2,25,H3,25,H3,50, H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,25,H3,50,H3,50,
        //不得真假 不做挣扎 不惧笑话
        H2,50,H1,25,M6,25,M6,25,H1,50, M6,25,H2,25,H1,25,M6,50,M6,25,H1,50,H1,50, H3,100,H3,25,H4,25,H3,50,H3,25,H2,50,H2,50,Z0,25,
        //我曾将青春翻涌成她 也曾指尖弹出盛夏 心之所动且就随缘去吧
        H1,50,H2,50,H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H2,50, H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H3,50,H3,50, H2,50,H1,25,M6,25,M6,25,H3,50,H3,50, H2,50,H1,25,M6,25,M6,25,H1,50,H1,50,H1,100,H1,100,Z0,10,
        //逆着光行走任风吹雨打吧
        M6,25,H3,50,H3,50,H2,50,H1,25,M6,25,M6,25,H3,50,H2,50,H1,25,M6,25,M6,25,H1,50,H1,50,H1,50,H1,100,Z0,25,
        //晚风吹起你鬓间的白发 抚平回忆留下的疤
        H1,50,H2,50,H2,25,H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H2,25, H3,25,H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,25,H3,50,H3,50,Z0,5,
        //你的眼中明暗交杂 一笑生花 暮色遮住你蹒跚的步伐
        H2,50,H1,25,M6,25,M6,25,H1,50,M6,25,H2,25,H1,25,M6,50,M6,50, H1,50,H1,50,H3,100,H3,25,H4,25,H3,50,H3,25,H2,50,H2,50,H1,100,H2,100,H3,100,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H2,50,
        //走进床头藏起的画 画中的你(低着头说话)
        H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H3,100,H3,50,H2,50,H1,25,M6,25,M6,25,H3,50,H3,50,H2,50,H1,25,M6,25,M6,25,H1,100,H1,50,Z0,5,
        //我仍 了 以爱之名你还愿意吗（衔接头有问题）
        H1,50,H2,50,H1,50,H1,100,M6,25,H3,50,H3,50,H2,50,H1,25,H6,25,H6,25,H3,50,H3,50,H2,50,H1,25,H6,25,H6,25,H1,50,H1,50,H1,100,Z0,5
    };