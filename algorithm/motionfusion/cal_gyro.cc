#include "cal_gyro.h"

#define GYRO_CALI_ZERO_BIAS_VARIANCE_THRESHOLD  (0.000001f)


uint8_t gyro_zero_bias_cal(att_info_t &att, const Vector3<float> gyro)
{
    static float bias_cnt = 0;
    static Vector3<float> gyro_bias_sum(0.0f, 0.0f, 0.0f);
    static uint64_t start_cal_time = 0;

    if (start_cal_time == 0)
    {
        start_cal_time = get_time_usec();

        printf("gyro start to cali\r\n");
    }

    if (get_time_usec() - start_cal_time > 10 * 1000 * 1000)
    {
        printf("gyro  cali over 10 sec, cal again\r\n");

        printf("att.gyro_variance: %10.7f\r\n", att.gyro_variance);

        start_cal_time = get_time_usec();
    }

    if (att.gyro_variance < GYRO_CALI_ZERO_BIAS_VARIANCE_THRESHOLD )
    {
        bias_cnt++;
        gyro_bias_sum += gyro;        
    }

    //1000次的平均值
    if (bias_cnt == 1000) 
    {
        att.gyro_bias = gyro_bias_sum / bias_cnt;

        fprintf(att.cfg_fp,"gyro_bias: %10.7f %10.7f %10.7f\n", att.gyro_bias.x, att.gyro_bias.y, att.gyro_bias.z);

        gyro_bias_sum.set(0.0f, 0.0f, 0.0f);
        bias_cnt = 0;
        start_cal_time = 0;

        printf("gyro cali ok!\r\n");

        printf("gyro bias: %10.7f %10.7f %10.7f\r\n", att.gyro_bias.x, att.gyro_bias.y, att.gyro_bias.z);

        return 1;
    }
    else
    {
        return 0;
    }
}