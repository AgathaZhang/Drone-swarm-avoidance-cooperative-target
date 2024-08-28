#ifndef SENSORFUSION6_H_
#define SENSORFUSION6_H_
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>
#include "hg_math.h"


typedef struct IMU_RAW
{
    int16_t acc[3];
    int16_t gyro[3];
    int16_t temp;    
} imu_raw_t;


typedef struct 
{
    //输出姿态角
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
    //姿态四元素
    float q0    = 1;
    float q1    = 0;
    float q2    = 0;
    float q3    = 0;
    //kalman滤波器	
    uint8_t filter_initted_flag = 0;
    Matrix<float, 4, 1> xk;
    Matrix<float, 4, 4> pk;
    Matrix<float, 4, 4> q;
	
    //叉积计算误差的累计积分
    Vector3<float> ex_int;
    //加速度零偏
    Vector3<float> acc_bias;
    //加速度计的尺度因子
    Vector3<float> acc_scale;
    //陀螺零偏
    Vector3<float> gyro_bias;
    //角速度计的尺度因子
    Vector3<float> gyro_scale;
    //加速度计方差
    float acc_variance;
    //角速度计方差
    float gyro_variance;
    //零偏矫正标志
    uint8_t acc_bias_calibrated;
    uint8_t gyro_bias_calibrated;
    //加速度校准状态
    uint8_t acc_cal_status;
    //零偏矫正命令标志
    uint8_t acc_cal_cmd;
    uint8_t gyro_cal_cmd;
    //调试文件
    FILE *fp = NULL;
    //配置参数保存文件
    FILE *cfg_fp = NULL;
}att_info_t;


typedef struct 
{
	uint64_t pts;
	float roll;
    float pitch;
    float yaw;
    float ground_distance;
    
    float rollspeed;
    float pitchspeed;
    float yawspeed;
} motion_fusion_t;

uint64_t get_time_usec(void);

uint8_t read_imu_cal_param(att_info_t &att);

void AttUpdate(imu_raw_t &data, att_info_t &att, float dt);

#endif /* SENSORFUSION6_H_ */