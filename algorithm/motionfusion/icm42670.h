#ifndef __ICM426xx_H_
#define __ICM426xx_H_

#include <stdint.h>

/* BANK0 */
#define ICM42670_DEVICE_CONFIG              0x01
#define ICM42670_DRIVE_CONFIG1              0x03
#define ICM42670_DRIVE_CONFIG2              0x04
#define ICM42670_DRIVE_CONFIG3              0x05
#define ICM42670_INT_CONFIG                 0x06
#define ICM42670_FIFO_CONFIG1               0x28
#define ICM42670_FIFO_CONFIG2               0x29
#define ICM42670_FIFO_CONFIG3               0x2A
#define ICM42670_TEMP_DATA1                 0x09
#define ICM42670_TEMP_DATA0                 0x0A
#define ICM42670_SIGNAL_PATH_RESET          0x02
#define ICM42670_INT_SOURCE0                0x2B
#define ICM42670_INT_SOURCE1                0x2C
#define ICM42670_INT_SOURCE3                0x2D
#define ICM42670_INT_SOURCE4                0x2E
#define ICM42670_WHO_AM_I                   0x75
#define BLK_SEL_W                           0x79
#define MADDR_W                             0x7a
#define M_W                                 0x7b
#define BLK_SEL_R                           0x7c
#define MADDR_R                             0x7d
#define M_R                                 0x7e

/* MREG1 */
#define ICM42670_TMST_CONFIG1_MREG1         0x00
#define ICM42670_FIFO_CONFIG5_MREG1         0x01
#define ICM42670_FIFO_CONFIG6_MREG1         0x02
#define ICM42670_FSYNC_CONFIG_MREG1         0x03
#define ICM42670_INT_CONFIG0_MREG1          0x04
#define ICM42670_INT_CONFIG1_MREG1          0x05
#define ICM42670_SENSOR_CONFIG3_MREG1       0x06
#define ICM42670_ST_CONFIG_MREG1            0x13
#define ICM42670_SELFTEST_MREG1             0x14
#define ICM42670_INTF_CONFIG6_MREG1         0x23
#define ICM42670_INTF_CONFIG10_MREG1        0x25
#define ICM42670_INTF_CONFIG7_MREG1         0x28
#define ICM42670_OTP_CONFIG_MREG1           0x2b
#define ICM42670_INT_SOURCE6_MREG1          0x2f
#define ICM42670_INT_SOURCE7_MREG1          0x30
#define ICM42670_INT_SOURCE8_MREG1          0x31
#define ICM42670_INT_SOURCE9_MREG1          0x32
#define ICM42670_INT_SOURCE10_MREG1         0x33
#define ICM42670_APEX_CONFIG2_MREG1         0x44
#define ICM42670_APEX_CONFIG3_MREG1         0x45
#define ICM42670_APEX_CONFIG4_MREG1         0x46
#define ICM42670_APEX_CONFIG5_MREG1         0x47
#define ICM42670_APEX_CONFIG9_MREG1         0x48
#define ICM42670_APEX_CONFIG10_MREG1        0x49
#define ICM42670_APEX_CONFIG11_MREG1        0x4a
#define ICM42670_ACCEL_WOM_X_THR_MREG1      0x4b
#define ICM42670_ACCEL_WOM_Y_THR_MREG1      0x4c
#define ICM42670_ACCEL_WOM_Z_THR_MREG1      0x4d
#define ICM42670_OFFSET_USER0_MREG1         0x4e
#define ICM42670_OFFSET_USER1_MREG1         0x4f
#define ICM42670_OFFSET_USER2_MREG1         0x50
#define ICM42670_OFFSET_USER3_MREG1         0x51
#define ICM42670_OFFSET_USER4_MREG1         0x52
#define ICM42670_OFFSET_USER5_MREG1         0x53
#define ICM42670_OFFSET_USER6_MREG1         0x54
#define ICM42670_OFFSET_USER7_MREG1         0x55
#define ICM42670_OFFSET_USER8_MREG1         0x56
#define ICM42670_ST_STATUS1_MREG1           0x63
#define ICM42670_ST_STATUS2_MREG1           0x64
#define ICM42670_FDR_CONFIG_MREG1           0x66
#define ICM42670_APEX_CONFIG12_MREG1        0x67

#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00  // default
 
#define GFS_2000DPS   0x00 // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
 
 
#define AODR_1600Hz   0x05
#define AODR_800Hz    0x06 // default
#define AODR_400Hz    0x07
#define AODR_200Hz    0x08
#define AODR_100Hz    0x09
#define AODR_50Hz     0x0A
#define AODR_25Hz     0x0B
#define AODR_12_5Hz   0x0C
#define AODR_6_25Hz   0x0D
#define AODR_3_125Hz  0x0E
#define AODR_1_5625Hz 0x0F
 
 
#define GODR_1600Hz  0x05
#define GODR_800Hz   0x06 // default
#define GODR_400Hz   0x07
#define GODR_200Hz   0x08
#define GODR_100Hz   0x09
#define GODR_50Hz    0x0A
#define GODR_25Hz    0x0B
#define GODR_12_5Hz  0x0C
 

#define ICM42670_GYRO_CONFIG0       0x20
#define ICM42670_ACCEL_CONFIG0      0x21
#define ICM42670_GYRO_CONFIG1       0x23
//#define ICM42670_GYRO_ACCEL_CONFIG0 0x52
#define ICM42670_ACCEL_CONFIG1      0x24
 
#define DEVICE_CONFIG     0x01
#define DRIVE_CONFIG3     0x05

#define ICM42670_TEMP_OUT_H 0x09
#define ICM42670_TEMP_OUT_L 0x10
#define ICM42670_ACCEL_XOUT_H 0x0B
#define ICM42670_ACCEL_XOUT_L 0x0C
#define ICM42670_ACCEL_YOUT_H 0x0D
#define ICM42670_ACCEL_YOUT_L 0x0E
#define ICM42670_ACCEL_ZOUT_H 0x0F
#define ICM42670_ACCEL_ZOUT_L 0x10
 
#define ICM42670_GYRO_XOUT_H 0x11
#define ICM42670_GYRO_XOUT_L 0x12
#define ICM42670_GYRO_YOUT_H 0x13
#define ICM42670_GYRO_YOUT_L 0x14
#define ICM42670_GYRO_ZOUT_H 0x15
#define ICM42670_GYRO_ZOUT_L 0x16
 
//#define SELF_TEST_CONFIG 0X70
 
#define ICM42670_PWR_MGMT0  0x1F //电源管理，典型值：0x00(正常启用)


#define ICM42670_DEG_PER_LSB_250  0.0076295109483482f  //((2 * 250.0) / 65536.0)
#define ICM42670_DEG_PER_LSB_500  0.0152587890625f     //((2 * 500.0) / 65536.0)
#define ICM42670_DEG_PER_LSB_1000 0.030517578125f      //((2 * 1000.0) / 65536.0)
#define ICM42670_DEG_PER_LSB_2000 0.06103515625f       //((2 * 2000.0) / 65536.0)

#define ICM42670_G_PER_LSB_2      0.00006103515625f    //((2 * 2) / 65536.0)
#define ICM42670_G_PER_LSB_4      0.0001220703125f     //((2 * 4) / 65536.0)
#define ICM42670_G_PER_LSB_8      0.000244140625f     //((2 * 8) / 65536.0)
#define ICM42670_G_PER_LSB_16     0.00048828125f       //((2 * 16) / 65536.0)


int IMUDEV_Open(const char* dev);
void IMUDEV_Close(int fd);

int ICM42670_Init(int fd);//(char *sensor_name);
int ICM42670_GetMotion6(int fd, uint8_t num, int16_t* ax, int16_t* ay, int16_t* az,
                        int16_t* gx, int16_t* gy, int16_t* gz, int16_t* temp);
float get_Icm42670GetAres(int fd);
float get_Icm42670GetGres(int fd);
//struct rt_spi_device* get_icm42670_dev(char* sensors_name);
#endif
