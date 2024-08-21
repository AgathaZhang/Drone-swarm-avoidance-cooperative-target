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
#include "mavlink_msg_formation_cmd_half.h"     // 接受来自飞控的命令字
#include "planning.hpp"

typedef enum _eDanceInfoCmd{
    DANCE_FILE_INFO,
    DANCE_FILE_PUSH_COMPLETE,
}eDanceInfoCmd;

typedef enum _eDanceInfoAck{
    DANCE_ACK_OK,
    DANCE_ACK_ERR,
}eDanceInfoAck;

class AlgorithmMng {
    // friend class FileDescriptorManager;             // 用友元在FileDescriptorManager 返回飞机总架次 
public:

    int ALL_DRONE_NUM = 100000;			            // 飞机总数
    int ID = 100000;                                // 补位目标ID
    pps moment = 0;                                 // 当前实时帧
    constraint limit;                               // 最大合速度 避碰半径 量化步长约束
    vec3d virtual_posi = {0, 0, 0};                 // 虚拟位置
    vec3d velocity = {0, 0, 0};                     // 实时速度
    vec3d pos_predict = {0, 0, 0};                  // 预测位置
    Guide_vector guider;                            // 制导向量 & 基于的实时帧
    CircularQueue queue;                            // 保证SDcard读写读写充盈

public:
    AlgorithmMng();
    ~AlgorithmMng();

    void start();
    void stop();

    void sendToPc(const char* data, int len);                       // 上发adb函数
    // void sendQrPosition(__mavlink_qrcode_t *msg);                // (弃用)
    void send_planningPosition(mavlink_auto_filling_dance_t *msg);  // 补位算法发送期望位置
    void onMavlinkMessage(const mavlink_message_t *message);        // 回调函数
    void handleMsgFromDrone(mavlink_message_t *msg);

    int DanceAdbAck(unsigned char cmd, char *filename, unsigned int size, char *md5);

    /** 补位函数 */
    // void comtocom(void);
    void inner_log(void);
    void init_target(void);
    void receive(void);
    void send_guidance_data(Guide_vector& guider);
    void loadInCycque(const pps& first_moment, CircularQueue& queue);
    void planning(CircularQueue& queue/*轨迹表*/, int& ID/*丢失的droneID*/, const vec3d& position/*当前位置*//*, Guide_vector& /*输出位置*/, const pps& moment/*时间戳*/, constraint limit/*飞机各类约束*/);    // 虚拟位置 速度乘以指导向量 = 实际位置 速度用的是打卡速度 
    void exception_handling();
    void Pos_estimator();              // 位置预测
    // void closureexpand(void);       // 单连通域闭包扩增

    /** 补位仿真 */
    // std::thread timeThread(timegoes, std::ref(moment));
    // std::thread consume(consumeInCycque, std::ref(moment), std::ref(queue));                      // 子线程用于剔除旧帧
    void Virtual_location(const Guide_vector& origin_guide/*当前指导向量*/, vec3d& virtual_posi/*当前虚拟位置*/, const pps& moment, const constraint limit);

private:
    /** 先导段*/   /** 避障段*/   /** 末端段*/ 
    bool guidance_phase = true;                     // 初始制导阶段
    int guidance_time = 2;                          // 初始制导上升时间 s
    double guidance_ascent_speed = 1.5;             // 初始上升速度 m/帧

    bool inversePlanning = false;                   // 本次路径是否反向
    int densimeter = 0;                             // 局部hole密度
    int failPlanning_count = 0;                     // 解反向次数    TODO 可以通过解终点方向判断解反向情况
    int bad_quadrantDrone_num = -1;                 // 解品质 连续倒退次数
    double solution_time = 0;                       // 单次解算时间
    double endpoint_distance = 1000;                // 当前距离终端位置的距离
    double end_scope = 1;                           // 结束避障阶段的距离条件 剩1s内到达终点
    double end_switch_dis = 3;                      // 切换至末端状态机的剩余距离
    
    /** 约束*/ 
    bool overtime = false;                          // 超时警告 直接退出线程
    bool depletion = false;                         // 单次规划期望全部耗尽指示
    bool termination = false;                       // 终止补位
    bool dataReady = false;                         // 数据流准备好
    bool guide_finish = false;                      // 路径全流程完成标志
    bool non_return_state_machine = false;          // 单向状态机止回阀
    bool is_send_dataInplanning = false;            // 规划阶段的发送线程控制状态字

    int cmd_switch;                                 // 程序切换命令字状态
    int cycbuffer_residue;                          // cycbuffer 剩余
    int sleep_time = 600;                           // 单次计算后的睡眠时间
    // int sleep_time = 2000;                          // 10m规划时单次计算后的睡眠时间
    int margin = 5;                                // 时空上的障碍飞机裕量
    int danceFrame_rate = 30;                       // 舞步帧速率
    int force_entern_endpoint = 10;                 // 强制进入末端舞步时间(测试用)
    double sleep_seconds;                           // 由上面sleep_time转换成的double秒

    std::mutex dataReady_mtx;                       // 数据流正常 启动阻塞锁(弃用)
    std::mutex mtx_position;                        // position & frame 时空数据锁
    std::mutex is_send_dataInplanning_cv_mtx;       // 发送子线程锁(弃用)
    std::condition_variable dataReady_cv;
    std::condition_variable is_send_dataInplanning_cv;  // 发送子线程条件变量
    
    // set3d view_matrix;                              // 测试soket用到
    // bool isSorted = false;                          // 测试soket用到 用于指示数据是否已排序
    // bool position_update = true;                    // 位置已更新
    // std::vector<vec3d> guide_soket;                 // 传输预测轨迹
    // std::mutex mtx_guide_soket;                     // 预测向量锁(弃用)
    // std::mutex changed;                             // monitor 互斥锁(弃用)
    // bool parameter_changed = false;                 // 参数改变检测标志
    // CircularQueue* queue = nullptr;                 // 初始化循环队列

    void ImuThread();
    void DroneThread();
    void CalAccThread();

private:
    /** 补位线程 */
    std::thread comtocomthread;
    std::thread logThread;
    std::thread receiveThread;
    std::thread timeThread;
    std::thread loaderThread;
    std::thread planningThread;
    std::thread VirtualdroneThread;
    std::thread send_dataInplanning;
    std::thread exception_handlingThread;


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
