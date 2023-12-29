#include "controller.h"
#include <math.h>

RC_DataTypeDef RC_CtrlData;
uint8_t RC_RxBuffer[18];

#ifndef M_PI
#define M_PI 3.1416
#endif

#define ABS(x) (x>0?x:-x)

// 计算距离原点的距离
static double calculateDistance(uint16_t x_t, uint16_t y_t) {
    double x = (x_t>1024?(x_t-1024):(1024-x_t));
    double y = (y_t>1024?(y_t-1024):(1024-y_t));
    return sqrt(x * x + y * y)*100;
}

// 计算相对于 x 轴正半轴的弧度（逆时针为正）
static double calculateRadian(uint16_t x_t, uint16_t y_t) {
    double x = x_t-1024;
    double y = (y_t-1024);
    double radian = atan2(y, x);
    if (radian < 0) {
        radian += 2*M_PI; // 将角度转换为正值
    }
    return radian;
}

// 将弧度约束在0-Π/2范围内
static double limitRadian(double radian){
    if(radian > M_PI)   radian = 2*M_PI-radian;
    if(radian > M_PI / 2)   radian = M_PI - radian;
    if(radian > M_PI / 4)   radian = M_PI/2 - radian;
    return radian;
}
// 获取拨杆状态
static uint8_t getPULLROD(void){
    uint8_t c = 0;
    /*解释：左  右
       中下上0  中下上0*/
    c = (0x10<<RC_CtrlData.rc.s1)|(0x01<<RC_CtrlData.rc.s2);
    return c;
}


/************************************************************
 * @brief 解析控制器数据
 * 
 * @param pData 数据缓冲区地址
************************************************************/
void RC_RecevieAnalysis(uint8_t *pData){
    if(pData == NULL)
    {
    return;
    }
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];

    RC_CtrlData.key.v = ((int16_t)pData[14]);

    RC_CtrlData.Ctrl.ROCKER_L_DEG=calculateRadian(RC_CtrlData.rc.ch2,RC_CtrlData.rc.ch3);
    RC_CtrlData.Ctrl.ROCKER_R_DEG=calculateRadian(RC_CtrlData.rc.ch0,RC_CtrlData.rc.ch1);
    RC_CtrlData.Ctrl.ROCKER_L=calculateDistance(RC_CtrlData.rc.ch2,RC_CtrlData.rc.ch3)/(660/cos(limitRadian(RC_CtrlData.Ctrl.ROCKER_L_DEG)));
    RC_CtrlData.Ctrl.ROCKER_R=calculateDistance(RC_CtrlData.rc.ch0,RC_CtrlData.rc.ch1)/(660/cos(limitRadian(RC_CtrlData.Ctrl.ROCKER_R_DEG)));
}

/************************************************************
 * @brief 将控制器摇杆数据解析为极坐标
************************************************************/
void RC_DataAnalyse_toPolar(void){
    /* 人造摇杆死区 */
    uint8_t val = 10;
    uint16_t* ch[4]={&RC_CtrlData.rc.ch0,&RC_CtrlData.rc.ch1,&RC_CtrlData.rc.ch2,&RC_CtrlData.rc.ch3};
    for(int i = 0; i < 4 ; i++) 
        if(*ch[i] < 1024+val && *ch[i] > 1024-val )   *ch[i] = 1024;
    
    /* 将摇杆数据转换为极坐标 */
    RC_CtrlData.Ctrl.ROCKER_L_DEG = calculateRadian(RC_CtrlData.rc.ch2, RC_CtrlData.rc.ch3);
    RC_CtrlData.Ctrl.ROCKER_R_DEG = calculateRadian(RC_CtrlData.rc.ch0, RC_CtrlData.rc.ch1);
    RC_CtrlData.Ctrl.ROCKER_L     = calculateDistance(RC_CtrlData.rc.ch2, RC_CtrlData.rc.ch3) / (660 / cos(limitRadian(RC_CtrlData.Ctrl.ROCKER_L_DEG)));
    RC_CtrlData.Ctrl.ROCKER_R     = calculateDistance(RC_CtrlData.rc.ch0, RC_CtrlData.rc.ch1) / (660 / cos(limitRadian(RC_CtrlData.Ctrl.ROCKER_R_DEG)));
}

/************************************************************
 * @brief 获取控制器数据值
 *        推度使用百分比表示，角度使用角度制表示
 * @param type 需要获取的数据类型
 * @return 返回值
************************************************************/
uint16_t RC_GetData(RC_DataType type){
    switch(type){
        case RC_ROCKER_L:
            return (uint16_t)RC_CtrlData.Ctrl.ROCKER_L;
        case RC_ROCKER_L_DEG:
            return (uint16_t)(RC_CtrlData.Ctrl.ROCKER_L_DEG * 180.0 / M_PI);
        case RC_ROCKER_R:
            return (uint16_t)RC_CtrlData.Ctrl.ROCKER_R;
        case RC_ROCKER_R_DEG:
            return (uint16_t)(RC_CtrlData.Ctrl.ROCKER_R_DEG * 180.0 / M_PI);
        case RC_PULLROD:
            return (uint16_t)getPULLROD();
        case RC_KEY:
            return (uint16_t)(RC_CtrlData.key.v);
        default:
            return 0;
    }
}

