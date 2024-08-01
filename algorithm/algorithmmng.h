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
#include "planning.hpp"


class AlgorithmMng {
public:
    std::mutex mtx_position;                        // position 位置&时间数据锁
    // std::mutex mtx_guide_soket;                     // 预测向量锁(弃用)
    // std::mutex changed;                          // monitor 互斥锁(弃用)
    // std::condition_variable cv;                  // 线程信号
    // bool parameter_changed = false;              // 参数改变检测标志
    // bool ready = false;
    // bool yes_change = false;
    int failplanning_count;
    int ID;                                         // 补位目标
    pps moment;                                     // 实时帧
    constraint limit;                               // 约束
    vec3d virtual_posi;                             // 虚拟位置
    std::vector<vec3d> guide;                       // 指导向量
    Guide_vector guider;                            // 封装 指导相邻 & 该向量生成基于的实时帧
    bool guide_finish = true;                       // 路径输出完成标志
    // set3d view_matrix;                              // 测试soket用到
    // bool isSorted = false;                          // 测试soket用到 用于指示数据是否已排序
    // bool position_update = true;                    // 位置已更新
    // std::vector<vec3d> guide_soket;                 // 传输预测轨迹
    // CircularQueue* queue = nullptr;                 // 初始化循环队列
    CircularQueue queue;

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
    void inner_log();
    void init_target();
    void receive();
    void planning(CircularQueue& queue/*轨迹表*/, int& ID/*丢失的droneID*/,const vec3d& position/*当前位置*/, Guide_vector& /*输出位置*/, const pps& moment/*时间戳*/, constraint limit/*飞机各类约束*/);
    void Virtual_location(const Guide_vector& origin_guide/*当前指导向量*/, vec3d& virtual_posi/*当前虚拟位置*/, const pps& moment, const constraint limit);

private:
    void ImuThread();
    void DroneThread();
    void CalAccThread();

private:
    /** 补位线程 */
    std::thread logThread;
    std::thread receiveThread;
    std::thread timeThread;
    std::thread loaderThread;
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
