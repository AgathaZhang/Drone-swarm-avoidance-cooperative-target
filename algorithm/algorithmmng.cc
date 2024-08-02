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



AlgorithmMng::AlgorithmMng() : queue(300)       // 在类初始化时 初始化sdcard中转buffer
{   
    init_target();
	mImuDevFd = -1;

	// mImuDevFd = IMUDEV_Open("/dev/spidev0.0");
    // ICM42670_Init(mImuDevFd);
}

AlgorithmMng::~AlgorithmMng() 
{
	// IMUDEV_Close(mImuDevFd);
}

void AlgorithmMng::start() {
    
    /** 补位新增*/
    logThread = std::thread(std::bind(&AlgorithmMng::inner_log, this));
    receiveThread = std::thread(std::bind(&AlgorithmMng::receive, this));           // 开启接收线程
    //TODO 这里应该阻塞，直到收到指定补位ID号且类成员明确被赋值后才进行读文件操作
    loaderThread = std::thread(loadInCycque, std::ref(moment), std::ref(queue));    // 加载缓冲池

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));                   // 给点时间让ram装载
    planningThread = std::thread(std::bind(&AlgorithmMng::planning, this, std::ref(queue), std::ref(ID)/*需要指定*/, std::ref(virtual_posi), std::ref(guider), std::ref(moment), limit));      // 输入当前位置 时间 输出期望位置guider(guide, moment)





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

    /** 补位新增*/
    receiveThread.join();



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
    // printf("Listen mavlink!!!!!!!!\n");
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
        
        case MAVLINK_MSG_ID_auto_filling_dance:
        {   
            mavlink_auto_filling_dance_t dance_cmd;
            mavlink_msg_auto_filling_dance_decode(msg, &dance_cmd);
            mtx_position.lock();
            virtual_posi.x = (double)dance_cmd.x;
            virtual_posi.y = (double)dance_cmd.y;
            virtual_posi.z = (double)dance_cmd.z;
            moment.frame = (unsigned int)dance_cmd.frame;
            mtx_position.unlock();
            // printf("x: %f,y: %fz: %fframe: %u\n", virtual_posi.x, virtual_posi.y, virtual_posi.z, moment.frame);

        }//TODO 
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

void AlgorithmMng::init_target()
{   
    // int result = system("/root/sethost.sh");
    // // int result = system("/root/test_zwz.sh");
    // if (result == 0) {printf("Make shell OTG host init success\n");}    // shell脚本成功执行
    // else {printf("Fail to shell OTG host\n");}                          // shell脚本执行失败
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // ID = 911;                       // SN from 1 instead of 0
    ID = 19;
    moment = {5095};                  // 第95帧开始丢
    virtual_posi = {0, 0, 0};
    // virtual_posi = {7, 23, 335};
    // virtual_posi = {-160, 70, 150};
    // queue = new CircularQueue(300); // 动态分配
    // printf("SUCCESS init\n");
}

void AlgorithmMng::inner_log() {
    // 删除已有的log.txt文件
    // std::remove("log.txt");

    while (true) {
        static int count = 0;
        // std::lock_guard<std::mutex> lock(mtx_position); // 确保线程安全
        printf("Recevinfo Count_NUM: %d x: %f,y: %fz: %fframe: %u\n",count, virtual_posi.x,virtual_posi.y,virtual_posi.z,moment.frame);
        // printf("Planninginfo %d ", num_get);

        // 使用 shell 命令将日志信息追加到文件末尾
        // char command[512];
        // snprintf(command, sizeof(command),
        //          "echo \"Recevinfo Count_NUM: %d x: %f, y: %f, z: %f, frame: %u\" >> log.txt && echo \"Planninginfo %d \" >> log.txt",
        //          count, virtual_posi.x, virtual_posi.y, virtual_posi.z, moment.frame);

        // system(command);

        count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 每500毫秒打印一次
    }
}


void AlgorithmMng::receive() {
    while (true)
    {   
        mavlink_message_t msg;
        handleMsgFromDrone(&msg);
        // std::this_thread::sleep_for(std::chrono::milliseconds(300)); 
    }
}