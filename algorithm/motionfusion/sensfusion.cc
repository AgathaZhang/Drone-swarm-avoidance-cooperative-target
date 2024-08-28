#include <math.h>

#include "sensfusion.h"
#include "hg_math.h"
#include "cal_gyro.h"
#include "cal_acc.h"

#define ACC_CALI_ZERO_BIAS_VARIANCE_THRESHOLD   (0.01f)


#define SENSOR_MAX_G            (16.0f)			   //constant g		// tobe fixed to 8g. but IMU need to  correct at the same time
#define SENSOR_MAX_W            (2000.0f)		       //deg/s
#define ACC_SCALE               (SENSOR_MAX_G/32768.0f)
#define GYRO_SCALE              (SENSOR_MAX_W/32768.0f/57.3f)


#define Kp                          (10.00f)
#define Ki                          (0.0001f)
#define ACC_SMOOTH_WINDOW_SIZE      (41)
#define GYRO_SMOOTH_WINDOW_SIZE     (41)

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


//加速度原始数据滑动窗口滤波
static float acc_data_smooth(Vector3<float> &acc)
{
	//在窗口没有填满的时候数据不可用 
	static uint8_t cnt = 0;
	static Vector3<float> buf[ACC_SMOOTH_WINDOW_SIZE];
	
	buf[cnt] = acc;
	buf[ACC_SMOOTH_WINDOW_SIZE -1] += acc;
	cnt++;
	cnt %= (ACC_SMOOTH_WINDOW_SIZE - 1);
	acc = buf[ACC_SMOOTH_WINDOW_SIZE - 1]/(ACC_SMOOTH_WINDOW_SIZE - 1);

    float variance = 0.0f;
    for (uint8_t i = 0; i < ACC_SMOOTH_WINDOW_SIZE - 1; i++)
    {
        float e = (buf[i] - acc).norm();
        variance = e * e;
    }
    variance /= (ACC_SMOOTH_WINDOW_SIZE - 1);

	buf[ACC_SMOOTH_WINDOW_SIZE -1] -= buf[cnt];

    return variance;
}

//角速度计原始数据滑动窗口滤波
static float gyro_data_smooth(Vector3<float> gyro)
{
	//在窗口没有填满的时候数据不可用
	static uint8_t cnt = 0;
	static Vector3<float> buf[GYRO_SMOOTH_WINDOW_SIZE];
	
	buf[cnt] = gyro;
	buf[GYRO_SMOOTH_WINDOW_SIZE - 1] += gyro;
	cnt++;
	cnt %= (GYRO_SMOOTH_WINDOW_SIZE - 1);
	gyro = buf[GYRO_SMOOTH_WINDOW_SIZE - 1]/(GYRO_SMOOTH_WINDOW_SIZE - 1);
    float variance = 0.0f;
    for (uint8_t i = 0; i < GYRO_SMOOTH_WINDOW_SIZE - 1; i++)
    {
        float e = (buf[i] - gyro).norm();
        variance = e * e;
    }
    variance /= (GYRO_SMOOTH_WINDOW_SIZE - 1);
	buf[GYRO_SMOOTH_WINDOW_SIZE - 1] -= buf[cnt];

    return variance;
}


uint8_t read_imu_cal_param(att_info_t &att)
{
    uint8_t ret = 0;

    uint64_t time = get_time_usec();

    uint8_t  read_cnt = 0;
    
    char name[32] = {0};
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // att.cfg_fp = fopen("./imu_cal_param.txt","r+"); 

    if (att.cfg_fp != NULL)
    {   
        //设置三秒钟超时
        //scanf每次读一行
        while (fscanf(att.cfg_fp, "%s %f %f %f\n",name, &x, &y, &z) != EOF)
        {
            if (get_time_usec() - time < 3 *1000 * 1000)
            {
                if (strcmp(name, "gyro_bias:") == 0)
                {
                    read_cnt |= 0x01;
                    att.gyro_bias.x = x;
                    att.gyro_bias.y = y;
                    att.gyro_bias.z = z;
                    printf("read imu cfg file gyro_bias is: %10.7f %10.7f %10.7f\n", att.gyro_bias.x, att.gyro_bias.y, att.gyro_bias.z); 
                }
                else if (strcmp(name, "acc_bias:") == 0)
                {
                    read_cnt |= 0x02;
                    att.acc_bias.x = x;
                    att.acc_bias.y = y;
                    att.acc_bias.z = z;
                    printf("read imu cfg file acc_bias is: %10.7f %10.7f %10.7f\n", att.acc_bias.x, att.acc_bias.y, att.acc_bias.z);
                }
                else if (strcmp(name, "acc_scale:")  == 0)
                {
                    read_cnt |= 0x04;
                    att.acc_scale.x = x;
                    att.acc_scale.y = y;
                    att.acc_scale.z = z;
                    printf("read imu cfg file acc_scale is: %10.7f %10.7f %10.7f\n", att.acc_scale.x, att.acc_scale.y, att.acc_scale.z);
                }
                else
                {
                    printf("read imu cfg file error\n"); 
                }

                if (read_cnt == 0x07)
                {
                    printf("read imu cfg file success\n"); 
                    ret = 1;
                    break;
                }
            }
            else
            {
                //读取超时了
                printf("read imu cfg file over time\n"); 
            }
        } 
    }
    else
    {
        printf("imu cfg_fp is null\n"); 
    }

    return ret;
}

void kalman_filter_init(att_info_t &att)
{
   att.filter_initted_flag = 1;

   att.xk(0, 0) = 0.0f; 
   att.xk(1, 0) = 0.0f;
   att.xk(2, 0) = att.gyro_bias.x;
   att.xk(3, 0) = att.gyro_bias.y;

   att.pk(0, 0) = 0.0003f;
   att.pk(1, 1) = 0.0003f;
   att.pk(2, 2) = 0.0000003f;
   att.pk(3, 3) = 0.0000003f;

   att.q(0, 0) = 0.0003f;
   att.q(1, 1) = 0.0003f;
   att.q(2, 2) = 0.0000003f;
   att.q(3, 3) = 0.0000003f;
}



//imu正交性没有处理
void AttUpdate(imu_raw_t &data, att_info_t &att, float dt)
{
    Vector3<float> acc;
    Vector3<float> gyro;
    acc.x = data.acc[0] * ACC_SCALE * CONSTANTS_ONE_G;
    acc.y = data.acc[1] * ACC_SCALE * CONSTANTS_ONE_G;
    acc.z = data.acc[2] * ACC_SCALE * CONSTANTS_ONE_G;
    gyro.x = data.gyro[0] * GYRO_SCALE;
    gyro.y = data.gyro[1] * GYRO_SCALE;
    gyro.z = data.gyro[2] * GYRO_SCALE;
    
    //错误数据直接返回
	if (acc.norm() < 1e-6f || gyro.norm() > 10.0f || dt > 50000) return;
    
    //加速度计平滑滤波
    att.acc_variance  = acc_data_smooth(acc);
    //陀螺滤波平滑并计算方差
    att.gyro_variance = gyro_data_smooth(gyro);
     
    //(没有校准或者收到校准命令)且陀螺方差小的时候计算零偏
    if ((att.gyro_bias_calibrated == 0) || att.gyro_cal_cmd == 1)
    {
        //校准没有完成直接返回
        if (gyro_zero_bias_cal(att, gyro) == 0) return;
        //校准完成清除校准命令和置位校准状态标志  
        att.gyro_cal_cmd = 0;
        att.gyro_bias_calibrated = 1;
    }

    //(没有校准或者收到校准命令)且加速度计方差小的时候计算零偏
    if ((att.acc_bias_calibrated == 0 || att.acc_cal_cmd == 1))
    {
        //校准没有完成直接返回
        if (acc_zero_bias_cal(att, acc) == 0) return;

        //校准完成清除校准命令和置位校准状态标志  
        att.acc_cal_cmd = 0;
        att.acc_bias_calibrated= 1;
    }
   	/*方法一
    gyro -= att.gyro_bias;
    acc  -= att.acc_bias;
    acc.x  *= att.acc_scale.x;
    acc.y  *= att.acc_scale.y;
    acc.z  *= att.acc_scale.z;

 	float q0q0 = att.q0*att.q0;
 	float q0q1 = att.q0*att.q1;
	float q0q2 = att.q0*att.q2;
	float q0q3 = att.q0*att.q3;
	float q1q1 = att.q1*att.q1;
 	float q1q2 = att.q1*att.q2;
 	float q1q3 = att.q1*att.q3;
	float q2q2 = att.q2*att.q2;
	float q2q3 = att.q2*att.q3;
	float q3q3 = att.q3*att.q3;

	//加速度计测量的重力方向(机体坐标系)
	acc.normalize();
    // //由四元素构建旋转矩阵
    // Matrix3<float> att_mat(Quat<float>(att.q0, att.q1, att.q2, att.q3));
    // //将加速度转换到n系
    // Vector3<float> acc_n = att_mat*acc;
    // //转换到n系后去除重力，得到运动加速度
    // acc_n -= Vector3<float>(0.0f, 0.0f, CONSTANTS_ONE_G);
    //加速度计只在低动态时，加速度测量值才具有矫正意义
    // if (att.acc_variance < 0.05f)
    {
        //四元数推出的实际重力方向(机体坐标系)
        Vector3<float> v_g(2*(q1q3 - q0q2), 2*(q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3);
        
        //叉积误差
        Vector3<float> e;
        e = acc.cross(v_g);

        //叉积误差积分为角速度
        att.ex_int += e * Ki;

        //角速度补偿
        gyro += e * Kp + att.ex_int;
    }
    
    //方法一
	//更新四元数
  	att.q0 = att.q0 + (-att.q1*gyro.x - att.q2*gyro.y - att.q3*gyro.z) * 0.5f *dt * 1e-6f;
  	att.q1 = att.q1 + (att.q0*gyro.x + att.q2*gyro.z - att.q3*gyro.y) * 0.5f *dt * 1e-6f;
  	att.q2 = att.q2 + (att.q0*gyro.y - att.q1*gyro.z + att.q3*gyro.x) * 0.5f *dt * 1e-6f;
  	att.q3 = att.q3 + (att.q0*gyro.z + att.q1*gyro.y - att.q2*gyro.x) * 0.5f *dt * 1e-6f;
	//单位化四元数
  	float norm = invSqrt(att.q0 * att.q0 + att.q1*att.q1 + att.q2*att.q2 + att.q3*att.q3);
  	att.q0 = att.q0 * norm;
  	att.q1 = att.q1 * norm;
  	att.q2 = att.q2 * norm;  
  	att.q3 = att.q3 * norm;

    //方法二
    // Quat<float> q(att.q0, att.q1, att.q2, att.q3);
    // Quat<float> dq = q.expq(gyro *dt * 1e-6f);
    // q = q * dq;
    // q.normalize();
	
    //方法一
	Quat<float> q(att.q0, att.q1, att.q2, att.q3);
    Vector3<float> vector = q.to_euler();
    att.yaw   = vector.z * 57.3f;
    att.pitch = vector.y * 57.3f;
    att.roll  = vector.x * 57.3f;
    
    //方法二
    //四元数反解欧拉角
	// att.yaw   = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f;
	// att.pitch = -asin(2.f * (q1q3 - q0q2))* 57.3f;
	// att.roll  = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f;
    */

    //方法二
    if (att.filter_initted_flag == 0)
    {
        kalman_filter_init(att);
    }

    Matrix<float, 4, 1> xk = att.xk;
    Matrix<float, 4, 4> pk = att.pk;
    Matrix<float, 4, 4> q  = att.q;

    Matrix<float, 2, 2> r;
    r(0, 0) = 0.0001f;
    r(1, 1) = 0.0001f;

    Matrix<float, 2, 1> ob;
    ob(0, 0) = atan2f(-acc.y, -acc.z);
    ob(1, 0) = atan2f(acc.x, -acc.z);

    Matrix<float, 4, 1> u;
    u(0, 0) = gyro.x;
    u(1, 0) = gyro.y;
    u(2, 0) = 0.0f;
    u(3, 0) = 0.0f;

    //状态转移矩阵A
    Matrix<float, 4, 4> a;
    a(0, 0) = 1.0f;
    a(0, 2) = -dt;
    a(1, 1) = 1.0f;
    a(1, 3) = -dt;
    a(2, 2) = 1.0f;
    a(3, 3) = 1.0f;
    //先验
    xk = a * xk + u * dt;
    //状态转移矩阵
    pk = a * pk * a.transpose() + q;
    //观测矩阵H
    Matrix<float, 2, 4> h;
    h(0, 0) = 1.0f;
    h(1, 1) = 1.0f;
    //卡尔曼增益K
    Matrix<float, 4, 2> k = pk * h.transpose() * (h * pk * h.transpose() + r).inverse(); 
    //后验
    xk = xk + k * (ob - h * xk);
    //更新后验方差
    Matrix<float, 4, 4> i;
    i.ones();
    pk = (i - k * h) * pk;

    att.xk = xk;
    att.pk = pk;
    att.q  = q;

    att.roll = att.xk(0, 0) * 57.3f;
    att.pitch  = att.xk(1, 0) * 57.3f;
    att.yaw   = 0.0f;
}
