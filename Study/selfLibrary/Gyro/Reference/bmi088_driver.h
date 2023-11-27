#ifndef __BMI088DRIVER_H
#define __BMI088DRIVER_H

#include "type_define.h"
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 1000

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
// #define BMI088_ACCEL_RANGE_6G
// #define BMI088_ACCEL_RANGE_12G
// #define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
// #define BMI088_GYRO_RANGE_1000
// #define BMI088_GYRO_RANGE_500
// #define BMI088_GYRO_RANGE_250
// #define BMI088_GYRO_RANGE_125

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f


#pragma pack(1)
typedef struct BMI088_RAW_DATA
{
    uint8_t status;
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} bmi088_raw_data_t;
#pragma pack()

typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    fp32 accel[3];
    fp32 temp;
    fp32 gyro[3];
    fp32 time;
} bmi088_real_data_t;

enum
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

uint8_t BMI088_Init(void);
extern bool_t bmi088_accel_init(void);
extern bool_t bmi088_gyro_init(void);
extern void BMI088_Offset_Cali();



void BMI088_read(fp32 gyro[3], fp32 accel[3]);
extern void get_BMI088_temperate(fp32 *temperate);
extern int BMI088_Init_flag;

void BMI088_Get_Data(void);

extern fp32 BMI088_ACCEL_SEN ;
extern fp32 BMI088_GYRO_SEN;
// extern mpu_data_t mpu_data;

#endif