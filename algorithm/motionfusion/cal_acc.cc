#include "cal_acc.h"
 #include <unistd.h>

//水平放置时水平轴加速度分量允许阈值
#define   ACC_CAL_ERR_THR      (1.0f)
//加速度计
#define   ACC_CAL_SUM_CNT      (1000)

//校准提示信息
const char *orientation_strs[6] = { "[forward]", "[back]", "[right]", "[left]", "[bottom]", "[up]" };
bool data_collected[6] = {false, false, false, false, false, false };
//校准过程开始的时间
uint64_t start_cal_time = 0;
//上次校准的时间
uint64_t last_cal_time = 0;
//上次打印数据的时间
uint64_t last_print_time = 0;
//校准过程某一面累计的加速度数据个数
uint16_t still_cnt[6] = {0};
//保存六个面静止时的值
Vector3<float> acc_sum[6];

uint64_t get_time_usec(void)
{
	struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

void acc_cal_init(void)
{
    //初始化一些参数
    last_print_time = 0;
    memset(still_cnt, 0, sizeof(still_cnt));

    printf("[accel calibrating]]\n");
    printf("Please keep the drone static\n");
    printf("****do the following operation****\r\n");
}


int detect_orientation(const Vector3<float> &acc)
{ 
    if (fabsf(acc.x - CONSTANTS_ONE_G) < ACC_CAL_ERR_THR &&
        fabsf(acc.y  < ACC_CAL_ERR_THR) &&
        fabsf(acc.z  < ACC_CAL_ERR_THR))
    {
        return 0;        // [ g, 0, 0 ]
    }

    if (fabsf(acc.x + CONSTANTS_ONE_G) < ACC_CAL_ERR_THR &&
        fabsf(acc.y) < ACC_CAL_ERR_THR &&
        fabsf(acc.z) < ACC_CAL_ERR_THR)
    {
        return 1;        // [ -g, 0, 0 ]
    }

    if (fabsf(acc.x) < ACC_CAL_ERR_THR &&
        fabsf(acc.y - CONSTANTS_ONE_G) < ACC_CAL_ERR_THR &&
        fabsf(acc.z) < ACC_CAL_ERR_THR)
    {
        return 2;        // [ 0, g, 0 ]
    }

    if (fabsf(acc.x) < ACC_CAL_ERR_THR &&
        fabsf(acc.y + CONSTANTS_ONE_G) < ACC_CAL_ERR_THR &&
        fabsf(acc.z) < ACC_CAL_ERR_THR)
    {
        return 3;        // [ 0, -g, 0 ]
    }

    if (fabsf(acc.x) < ACC_CAL_ERR_THR &&
        fabsf(acc.y) < ACC_CAL_ERR_THR &&
        fabsf(acc.z - CONSTANTS_ONE_G) < ACC_CAL_ERR_THR)
    {
        return 4;        // [ 0, 0, g ]
    }

    if (fabsf(acc.x) < ACC_CAL_ERR_THR &&
        fabsf(acc.y) < ACC_CAL_ERR_THR &&
        fabsf(acc.z + CONSTANTS_ONE_G) < ACC_CAL_ERR_THR)
    {
        return 5;        // [ 0, 0, -g ]
    }
    return -1;   // Can't detect orientation
}


//计算校准值
void calculate_cal_values(att_info_t &att)
{
    /* calculate offsets */
    att.acc_bias.x = (acc_sum[0] + acc_sum[1]).x * 0.5f;
    att.acc_bias.y = (acc_sum[2] + acc_sum[3]).y * 0.5f;
    att.acc_bias.z = (acc_sum[4] + acc_sum[5]).z * 0.5f;
    
    /* fill matrix A for linear equations system*/
    Matrix3<float> mat_a;

    for (uint8_t i = 0; i < 3; i++)
    {
        mat_a.set_row(i, acc_sum[i * 2] - att.acc_bias);
    }
    mat_a = mat_a.inverse();

    att.acc_scale.x = (mat_a * CONSTANTS_ONE_G).m[0][0];
    att.acc_scale.y = (mat_a * CONSTANTS_ONE_G).m[1][1];
    att.acc_scale.z = (mat_a * CONSTANTS_ONE_G).m[2][2];
}


uint8_t acc_zero_bias_cal(att_info_t &att, const Vector3<float> &acc)
{
    if (start_cal_time == 0)
    {
        start_cal_time = get_time_usec();
        //打印提示信息
        acc_cal_init();

        //为了和产测上位机校准时的状态对应，开始校准
        att.acc_cal_status = 38;    
    }
    uint64_t current_time = get_time_usec();

    if(current_time - last_print_time > 2000000)
    {
        last_print_time = current_time;

        printf("\n[ACAL]not calibrated: %s%s%s%s%s%s\r\n",
                   (!data_collected[5]) ? "[  up   ]" : "[--finished--]",
                   (!data_collected[4]) ? "[bottom ]" : "[--finished--]",
                   (!data_collected[0]) ? "[forward]" : "[--finished--]",
                   (!data_collected[1]) ? "[ back  ]" : "[--finished--]",
                   (!data_collected[3]) ? "[ left  ]" : "[--finished--]",
                   (!data_collected[2]) ? "[ right ]" : "[--finished--]");
    }

    //判断加速度计的方差检测加速度计是否处于静止状态
    if (att.acc_variance < 0.01f)
    {
        int index = detect_orientation(acc);
        //
        if (index >= 0 && index <= 5)
        {
            if (still_cnt[index] < ACC_CAL_SUM_CNT)
            {
                still_cnt[index]++;
                acc_sum[index] += acc;
            }
            else if (data_collected[index] == false)
            {
                acc_sum[index] /= ACC_CAL_SUM_CNT * 1.0f;
                data_collected[index] = true;

                //为了和产测上位机校准时的状态对应，请保持静止
                att.acc_cal_status = 40 + index;                      

            } 
            else if (current_time - last_print_time > 1000000)
            {
                last_print_time =  current_time;

                printf("[ACAL]请将IMU另一面水平放置!\r\n");         
            }             
        }


        uint8_t cal_finish_flag = 0;
        for (uint8_t i = 0; i < 6; i++)
        {
            cal_finish_flag += (data_collected[i] == true? 1:0);
        }

        if (cal_finish_flag == 6)
        {
            calculate_cal_values(att);
            //清除该参数就够了
            start_cal_time = 0;
            //清除计数值
            memset(still_cnt, 0, sizeof(still_cnt));

            att.acc_bias_calibrated = 1;  
            //提示全部校准完毕
            printf("\n[ACAL]not calibrated: %s%s%s%s%s%s\r\n",
                   (!data_collected[5]) ? "[  up   ]" : "[--finished--]",
                   (!data_collected[4]) ? "[bottom ]" : "[--finished--]",
                   (!data_collected[0]) ? "[forward]" : "[--finished--]",
                   (!data_collected[1]) ? "[ back  ]" : "[--finished--]",
                   (!data_collected[3]) ? "[ left  ]" : "[--finished--]",
                   (!data_collected[2]) ? "[ right ]" : "[--finished--]");

            printf("[ACAL]IMU 校准完毕!\r\n"); 

            printf("[ACAL]acc_bias: %10.7f  %10.7f  %10.7f\n", att.acc_bias.x, att.acc_bias.y, att.acc_bias.z); 
            fprintf(att.cfg_fp,"acc_bias: %10.7f  %10.7f  %10.7f\n",att.acc_bias.x, att.acc_bias.y, att.acc_bias.z);
            printf("[ACAL]IMU scale: %10.7f  %10.7f  %10.7f\n", att.acc_scale.x, att.acc_scale.y, att.acc_scale.z);
            fprintf(att.cfg_fp,"acc_scale: %10.7f %10.7f %10.7f\n", att.acc_scale.x, att.acc_scale.y, att.acc_scale.z);

            fclose(att.cfg_fp);
            
            //为了和产测上位机校准时的状态对应，校准完成
            att.acc_cal_status = 46;                    
            
            return 1;
        }
    }
    else
    {
        //避免打印太快
        if (current_time - last_print_time > 1000000)
        {
            last_print_time =  current_time;

            printf("[ACAL]请将IMU任一面静止水平放置!\r\n"); 
            //为了和产测上位机校准时的状态对应，请保持静止
            att.acc_cal_status = 39;           
        }
    } 
    return 0;
}

