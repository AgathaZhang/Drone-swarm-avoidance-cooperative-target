#include "algorithmmng.h"

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "uart.h"
#include "icm42670.h"
#include "app_mavlink.h"



AlgorithmMng::AlgorithmMng() 
{
	mImuDevFd = -1;

	// mImuDevFd = IMUDEV_Open("/dev/spidev0.0");
    // ICM42670_Init(mImuDevFd);
}

AlgorithmMng::~AlgorithmMng() 
{
	// IMUDEV_Close(mImuDevFd);
}

void AlgorithmMng::start() {
	// mImuStatus = true;
    // mImuThread = std::thread(std::bind(&AlgorithmMng::ImuThread, this));

	mDroneStatus = true;
    mDroneThread = std::thread(std::bind(&AlgorithmMng::DroneThread, this));
 
    // mCalAccThreadStatus = true;
    // mCalAccThread = std::thread(std::bind(&AlgorithmMng::CalAccThread, this));

    // mAppMvlinkStatus = true;
    // mAppMavlinkThread = std::thread(std::bind(&app_mavlink_main));

    mAdbServer = make_shared<AdbServer>();
    mAdbServer->setMavlinkMessageCallback(std::bind(&AlgorithmMng::onMavlinkMessage,this,_1));
    mAdbServer->start();
}

void AlgorithmMng::stop() {
    // mImuStatus = false;
    // mImuThread.join();

    mDroneStatus = false;
    mDroneThread.join();

	mAdbServer->stop();
}

void AlgorithmMng::sendToPc(const char *data, int len)
{
    // adb send flow
    if (mAdbServer && true == mAdbServer->getAppConnectionStatus()) 
    {
        mAdbServer->send(data, len);
    }
    else
    {
        printf("mAdbServer->send failed\n");
    }
}

void AlgorithmMng::CalAccThread()
{
    uint16_t cmd_len = 0;

    uint8_t send_flag = 0;

    mavlink_message_t msg;

    char text[128] = {0};

    printf("CalAccThread run\n");

    while(mCalAccThreadStatus)
    {
        if (this->att.acc_cal_cmd)
        {
            if (att.acc_cal_status == 38)
            {
                sprintf(text, "acc_cal_start");
                //提示开始校准
                cmd_len = mavlink_msg_statustext_pack(3, 5, &msg, 38, MAV_SEVERITY_DEBUG, text);

                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
            }
            else if (att.acc_cal_status == 39)
            {
                //提示保持静止
                sprintf(text, "acc_cal_hold_still");
                //提示开始校准
                cmd_len = mavlink_msg_statustext_pack(3, 5, &msg, 39, MAV_SEVERITY_DEBUG, text);

                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
            }
        
            else if (att.acc_cal_status >= 40 && att.acc_cal_status <= 45)
            {
                send_flag |= (0x01<<(att.acc_cal_status - 40));
                //提示哪个面的校准状态
                sprintf(text, "acc_cal_%d_ok", att.acc_cal_status - 40);
                //提示开始校准
                cmd_len = mavlink_msg_statustext_pack(3, 5, &msg, 40, MAV_SEVERITY_DEBUG, text);

                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
                sendToPc(reinterpret_cast<char *>(&msg), cmd_len);

            }
        }
        // 校准结束发送消息给产测上位机
        else if (att.acc_cal_status == 46)
        {

            uint8_t i = 0;
            while ((send_flag >>i) & 0x01)
            {
                  i++;
            }
            //提示最后一个面的校准状态
            sprintf(text, "acc_cal_%d_ok", i);
            //提示开始校准
            cmd_len = mavlink_msg_statustext_pack(3, 5, &msg, 40, MAV_SEVERITY_DEBUG, text);

            sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
            sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
            sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
            sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
            sendToPc(reinterpret_cast<char *>(&msg), cmd_len);

            usleep(1000*1000);

            //提示校准完成
            sprintf(text, "acc_cal_passed");
            //提示开始校准
            cmd_len = mavlink_msg_statustext_pack(3, 5, &msg, 42, MAV_SEVERITY_DEBUG, text);

            sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
            sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
            sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
            sendToPc(reinterpret_cast<char *>(&msg), cmd_len);
            sendToPc(reinterpret_cast<char *>(&msg), cmd_len);

            att.acc_cal_status = 0;

        }

        usleep(1000*1000);
    }
    printf("CalAccThread exit\n");
}

void AlgorithmMng::ImuThread()
{
    imu_raw_t imu = {0};

    uint64_t last_run_time = 0;

    uint64_t last_debug_time = 0;
    //默认以读方式打开文件
    att.cfg_fp = fopen("./imu_cal_param.txt","r+");     

    if (att.cfg_fp == NULL)
    {
        printf("cfg_fp fopen failed\n");
        return ;
    }
    //读取IMU的校准参数
    if (read_imu_cal_param(att))
    {
        att.acc_bias_calibrated = 1;

        att.gyro_bias_calibrated = 1;

        printf("imu_cal_param read ok\n");
    }
    else
    {
        att.acc_bias_calibrated = 0;

        att.gyro_bias_calibrated = 0;

        printf("imu_cal_param read fail\n");
    }

    char debug_buff[128] = {0};

	while(mImuStatus)
	{
		ICM42670_GetMotion6(mImuDevFd, 0, &imu.acc[0], &imu.acc[1], &imu.acc[2], &imu.gyro[0], &imu.gyro[1], &imu.gyro[2], &imu.temp);
		// printf("data  %6d %6d %6d %6d %6d %6d  %6d\n", imu.acc[0], imu.acc[1], imu.acc[2], imu.gyro[0], imu.gyro[1], imu.gyro[2], imu.temp);
		float  temp = imu.acc[0];
        imu.acc[0] = -imu.acc[2];
        imu.acc[1] = imu.acc[1];
        imu.acc[2] = temp;
        temp = imu.gyro[0];
        imu.gyro[0] = -imu.gyro[2];
        imu.gyro[1] = imu.gyro[1];
        imu.gyro[2] = temp;

        uint64_t current_time = get_time_usec();
        AttUpdate(imu, att, (current_time - last_run_time) * 1e-6f);
        last_run_time = current_time;

        if (att.acc_bias_calibrated == 1 /*&& current_time - last_debug_time > 1000 * 1000*/)
        {
            last_debug_time = current_time;
            printf("att  %10.7f %10.7f %10.7f\n", att.roll, att.pitch, att.yaw);         
        }

		usleep(25*100);
	}
}

//adb串口的回调函数
void  AlgorithmMng::onMavlinkMessage(const mavlink_message_t *msg)
{
	switch (msg->msgid)
	{
		case MAVLINK_MSG_ID_REPORT_STATS:
		{
			break;	
		}

		case MAVLINK_MSG_ID_ATTITUDE:
		{
			mavlink_attitude_t atti;
			mavlink_msg_attitude_decode(msg, &atti);

			break;	
		}
        case MAVLINK_MSG_ID_FORMATION_CMD:
        {
            mavlink_formation_cmd_t format_cmd;
            mavlink_msg_formation_cmd_decode(msg, &format_cmd);
            
            //接收到产测上位机的校准加速度计的命令
            if (format_cmd.cmd == 21)
            {
                this->att.acc_cal_cmd = 1;

                printf("mavlink received cal acc cmd\n");
            }
            break;
        }
        case MAVLINK_MSG_ID_auto_filling_dance:
        {   
            printf("recev !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            mavlink_auto_filling_dance_t dance_cmd;
            mavlink_msg_auto_filling_dance_decode(msg, &dance_cmd);

        }//TODO 
		default :
			break;
	}	
}

const uint8_t *dd = (const uint8_t *)"console_mavlink()\n";	// 18
void AlgorithmMng::DroneThread()
{
    int ret = 0;
    fd_set fd_read;
    struct timeval timeout;

    char mavlink_data[256] = {0};
    mavlink_status_t status;
    mavlink_message_t msg;

	mDroneDevFd = uart_open("/dev/ttyACM0", 460800);
	if (mDroneDevFd == -1) {
		printf("open drone_uart[/dev/ttyACM0] err");
		return;
	}

	while(mDroneStatus)
	{
        FD_ZERO(&fd_read);
        FD_SET(mDroneDevFd, &fd_read);
        timeout.tv_sec = 3;//0;
        timeout.tv_usec = 0;//200000; //200ms

        ret = select(mDroneDevFd+1, &fd_read, NULL, NULL, &timeout);
        if (ret < 0) {
            printf("drone_uart[/dev/ttyACM0] select err...");
        } else if (ret == 0) {
            //SPDLOG_INFO("drone_uart[/dev/ttyS4] not connected...");
            //SendMsgAckToApp(MAV_PLANE_CMD_DEVINFO, 0, CMD_ERR);
			printf("drone_uart[/dev/ttyACM0] not connected...");
			uart_send(mDroneDevFd, dd, 18);
		} else {

			if ( FD_ISSET(mDroneDevFd, &fd_read) > 0 ) {
				int s_ret = read(mDroneDevFd, mavlink_data, 256);
				//printf("%d \n", s_ret);               

				for (int i=0; i<s_ret; i++) {
					if ( mavlink_parse_char(MAVLINK_COMM_0, mavlink_data[i], &msg, &status) ) {
						handleMsgFromDrone(&msg);
					}
				}
			}
		}
	}

	uart_close(mDroneDevFd);
}

void AlgorithmMng::handleMsgFromDrone(mavlink_message_t *msg)
{
	switch (msg->msgid)
	{
		case MAVLINK_MSG_ID_REPORT_STATS:
		{
			//printf("MAVLINK_MSG_ID_REPORT_STATS\n");
			break;	
		}

		case MAVLINK_MSG_ID_ATTITUDE:
		{
			mavlink_attitude_t atti;
			mavlink_msg_attitude_decode(msg, &atti);

			//printf("ground_distance = %d\n", atti.ground_distance);
			break;	
		}
		default :
			break;
	}	
}

void AlgorithmMng::sendQrPosition(__mavlink_qrcode_t *msg)
{
    mavlink_message_t mavlink;
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
    uint16_t buf_size;
	
	mavlink_msg_qrcode_encode(0, 0, &mavlink, msg);
	buf_size = mavlink_msg_to_send_buffer(buf, &mavlink);

	uart_send(mDroneDevFd, buf, buf_size);
}

/**  float x;  
float y; 
float z; 
uint32_t frame; 
uint8_t reserved[5]; 
mavlink_auto_filling_dance_t;
*/

void AlgorithmMng::send_planningPosition(mavlink_auto_filling_dance_t *msg)
{
    mavlink_message_t mavlink;
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
    uint16_t buf_size;
	
	mavlink_msg_bwcode_encode(0, 0, &mavlink, msg);
	buf_size = mavlink_msg_to_send_buffer(buf, &mavlink);
    uart_send(mDroneDevFd, buf, buf_size);
	// mAdbServer->send(reinterpret_cast<const char*>(buf), static_cast<int>(buf_size));
}