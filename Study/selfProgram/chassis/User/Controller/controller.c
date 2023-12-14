#include "controller.h"
#include <math.h>
#include "motor.h"
#include "gyro.h"
#include "gimbal.h"
#include "chassis.h"

RC_DataTypeDef RC_CtrlData;
uint8_t RC_RxBuffer[18];
// float t_speed = 0; // 测试PID用的参数

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

    /* 将摇杆数据转换为极坐标 */
    RC_CtrlData.Ctrl.ROCKER_L_DEG = calculateRadian(RC_CtrlData.rc.ch2, RC_CtrlData.rc.ch3);
    RC_CtrlData.Ctrl.ROCKER_R_DEG = calculateRadian(RC_CtrlData.rc.ch0, RC_CtrlData.rc.ch1);
    RC_CtrlData.Ctrl.ROCKER_L     = calculateDistance(RC_CtrlData.rc.ch2, RC_CtrlData.rc.ch3) / (660 / cos(limitRadian(RC_CtrlData.Ctrl.ROCKER_L_DEG)));
    RC_CtrlData.Ctrl.ROCKER_R     = calculateDistance(RC_CtrlData.rc.ch0, RC_CtrlData.rc.ch1) / (660 / cos(limitRadian(RC_CtrlData.Ctrl.ROCKER_R_DEG)));

    /* 人造死区 */
    // 摇杆归中时角度并不是完全的0……
    if (RC_CtrlData.Ctrl.ROCKER_L == 0) RC_CtrlData.Ctrl.ROCKER_L_DEG = 0;
    if (RC_CtrlData.Ctrl.ROCKER_R == 0) RC_CtrlData.Ctrl.ROCKER_R_DEG = 0;
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
void RC_CtrlChassis(void)
{
    static uint16_t L_speed = 1500; // 低速模式转速
    static uint16_t H_speed = 3000; // 高速模式转速
    static uint8_t PULLROD_flag;    // 拨杆
    PULLROD_flag = RC_GetData(RC_PULLROD);

    // // 检测左0右1摇杆是否归零 跳变沿标志位（当归零时触发一次）,0为此前为0，1为此前大于0，2为上升沿，4为下降沿
    // static uint8_t JumpEdge_flag[2];
    // /* 跳变沿判断 */
    // JumpEdge_flag[0]=JumpEdge_flag[0]<2?(((JumpEdge_flag[0]==1)^(RC_GetData(RC_ROCKER_L)>0))?0x02<<JumpEdge_flag[0]:JumpEdge_flag[0]):(RC_GetData(RC_ROCKER_L)>0?1:0);
    // JumpEdge_flag[1]=JumpEdge_flag[1]<2?(((JumpEdge_flag[1]==1)^(RC_GetData(RC_ROCKER_R)>0))?0x02<<JumpEdge_flag[1]:JumpEdge_flag[1]):(RC_GetData(RC_ROCKER_R)>0?1:0);

    /* 控制器操作 */
    gyro_calData();
    switch (PULLROD_flag) {
        case 0b00100010: // 双上 云台底盘能被正常控制，且底盘速度为正常模式
            chasis_run_with_gimbal(L_speed * RC_GetData(RC_ROCKER_L) / 100, RC_CtrlData.Ctrl.ROCKER_L_DEG);
            gimbal_move_by_controller();
            // if(JumpEdge_flag[0]==4) motor[1].stop_flag=motor[2].stop_flag=motor[3].stop_flag=motor[4].stop_flag=1;
            // if(JumpEdge_flag[1]==4) gimbal_yaw_motor.stop_flag=1;
            break;
        case 0b01000100: // 双下 云台无力，底盘速度保持为 0
            chasis_run(0, 0, 0);
            gimbal_down();
            break;
        case 0b00101000: // 左上右中 底盘速度为高速模式
            chasis_run_with_gimbal(H_speed * RC_GetData(RC_ROCKER_L) / 100, RC_CtrlData.Ctrl.ROCKER_L_DEG);
            gimbal_move_by_controller();
            break;
        case 0b00100100: // 左上右下 底盘小陀螺模式
            chasis_run(L_speed * RC_GetData(RC_ROCKER_L) / 100, RC_CtrlData.Ctrl.ROCKER_L_DEG, 3000);
            gimbal_move_by_controller();
            break;
        case 0b10000010: // 左中右上 自由移动云台
            chasis_run(0, 0, 0);
            gimbal_move_by_controller();
            // PTZ_turn_bySpeed(9,t_speed);//调试pitch用
            break;
        case 0b10001000: // 双中 重新设置方向
            gimbal_down();
            gimbal_location_set();
            break;
        // case 0b10000100: // 左中右下 测试Yaw云台（正弦速度曲线）
        //     gimbalFLAG = gimbal_ON;
        //     chasis_run(0, 0, 0, 0);
        //     float t=HAL_GetTick();
        //     t_speed = 0.785*sin(1.884*t/1000)+1.305;
        //     t_speed *=2;
        //     PTZ_turn_bySpeed(5,t_speed);
        //     gyro_setYaw();
        //     break;
    }
}
