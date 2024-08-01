#ifndef __ALGORITHMMNG_H_
#define __ALGORITHMMNG_H_

#include <thread>
#include <functional>

#include "adbserver.h"
#include "mavlink_msg_report_stats.h"
#include "mavlink_msg_attitude.h"
#include "mavlink_msg_qrcode_position.h"
#include "mavlink_msg_formation_cmd.h"
#include "mavlink_msg_statustext.h"
#include "motionfusion/sensfusion.h"
#include "mavlink_msg_auto_filling_dance.h"     // 补位新加协议
// #include "formation.hpp"
// #include "planning.hpp"


class AlgorithmMng {

public:
    AlgorithmMng();
    ~AlgorithmMng();

    void start();
    void stop();

    void sendToPc(const char* data, int len);                       // 上发adb函数
    void sendQrPosition(__mavlink_qrcode_t *msg);
    void send_planningPosition(mavlink_auto_filling_dance_t *msg);  // 补位算法发送期望位置
    void onMavlinkMessage(const mavlink_message_t *message);        // 回调函数 TODO 待修改
    void handleMsgFromDrone(mavlink_message_t *msg);

    /** 补位函数 */
    void receive();

private:
    void ImuThread();
    void DroneThread();
    void CalAccThread();

private:
    /** 补位线程 */
    std::thread timeThread;
    std::thread receiveThread;
    std::thread loader;
    std::thread planningThread;
    std::thread VirtualdroneThread;


    /** 其他线程*/
    std::thread mImuThread;
    bool mImuStatus;
    int mImuDevFd;

    std::thread mDroneThread;
    bool mDroneStatus;
    int mDroneDevFd;

    std::thread mCalAccThread;
    bool mCalAccThreadStatus;

    std::thread mAppMavlinkThread;
    bool mAppMvlinkStatus;

    att_info_t  att;

    shared_ptr<AdbServer>  mAdbServer;
};

#endif  //!__MSG_MANAGER_H_
