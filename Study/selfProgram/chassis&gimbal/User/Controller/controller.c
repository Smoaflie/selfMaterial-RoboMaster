#include "controller.h"
#include <math.h>
#include "motor.h"
#include "gyro.h"
#include "gimbal.h"
#include "chassis.h"

RC_DataTypeDef RC_CtrlData;
uint8_t RC_RxBuffer[18];


// 计算距离原点的距离
static double calculateDistance(uint16_t x_t, uint16_t y_t)
{
    double x = (x_t > 1024 ? (x_t - 1024) : (1024 - x_t));
    double y = (y_t > 1024 ? (y_t - 1024) : (1024 - y_t));
    return sqrt(x * x + y * y) * 100;
}

// 计算相对于 y 轴正半轴的弧度（逆时针为正）
static double calculateRadian(uint16_t x_t, uint16_t y_t)
{
    double x      = x_t - 1024;
    double y      = (y_t - 1024);
    double radian = atan2(y, x) - M_PI / 2.0;
    if (radian < 0) {
        radian += 2 * M_PI; // 将角度转换为正值
    }
    return radian;
}

// 将弧度约束在0-Π/2范围内
static double limitRadian(double radian)
{
    if (radian > M_PI) radian = 2 * M_PI - radian;
    if (radian > M_PI / 2) radian = M_PI - radian;
    if (radian > M_PI / 4) radian = M_PI / 2 - radian;
    return radian;
}
// 获取拨杆状态
static uint8_t getPULLROD(void)
{
    uint8_t c = 0;
    /*解释：左  右
       中下上0  中下上0*/
    c = (0x10 << RC_CtrlData.rc.s1) | (0x01 << RC_CtrlData.rc.s2);
    return c;
}

/************************************************************
 * @brief 解析控制器数据
 *
 * @param pData 数据缓冲区地址
 ************************************************************/
void RC_RecevieAnalysis(uint8_t *pData)
{
    if (pData == NULL) {
        return;
    }
    /* 解析控制器数据 */
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;
    RC_CtrlData.rc.s1  = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2  = ((pData[5] >> 4) & 0x0003);

    RC_CtrlData.mouse.x       = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y       = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z       = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];

    RC_CtrlData.key.v = ((int16_t)pData[14]);
}

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
 * @return uint16_t 返回值
 ************************************************************/
uint16_t RC_GetData(RC_DataType type)
{
    switch (type) {
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

/* 通过控制器设置电机速度 */
void RC_CtrlCar(void)
{

    static uint16_t L_speed = 1500; // 低速模式转速
    // L_speed = 1499; 
    static uint16_t H_speed = 3000; // 高速模式转速
    static uint8_t PULLROD_flag;    // 拨杆
    PULLROD_flag = RC_GetData(RC_PULLROD);

    float t=HAL_GetTick();
    float t_speed = 0.785*sin(1.884*t/1000)+1.305;

    /* 控制器操作 */
    switch (PULLROD_flag) {
        case 0b00100010: // 双上 云台底盘能被正常控制，且底盘速度为正常模式
            chassis_run_with_gimbal(L_speed);
            gimbal_move_by_controller();
            break;
        case 0b01000100: // 双下 云台无力，底盘速度保持为 0
            chassis_off();
            gimbal_down();
            break;
        case 0b00101000: // 左上右中 底盘速度为高速模式
            chassis_run_with_gimbal(H_speed);
            gimbal_move_by_controller();
            break;
        case 0b00100100: // 左上右下 底盘小陀螺模式
            chassis_top(L_speed,3000);
            gimbal_move_by_controller();
            break;
        case 0b10000010: // 左中右上 自由移动云台
            chassis_off();
            gimbal_move_by_controller();
            break;
        case 0b10001000: // 双中 重新设置方向
            chassis_off();
            gimbal_down();
            gimbal_location_reset();
            break;
    }
}

/* 云台任务 放在freertos中 */
void controller_task(void){
    RC_DataAnalyse_toPolar();

    RC_CtrlCar();
}

