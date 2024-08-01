#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <string>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
// #include <winsock2.h>
// #include <ws2tcpip.h>             // && 临时注释 linux上需要修改此项为兼容的库
// #include <sys/socket.h>           // && 临时注释
// #include <arpa/inet.h>
#include "covering_algorithm/formation.hpp"
#include "covering_algorithm/planning.hpp"
// #pragma comment(lib, "Ws2_32.lib")
#include "algorithmmng.h"


std::mutex mtx_position;                        // position 位置数据锁
std::mutex mtx_guide_soket;                     // 预测向量锁(弃用)
// std::mutex changed;                          // monitor 互斥锁(弃用)
// std::condition_variable cv;                  // 线程信号
// bool parameter_changed = false;              // 参数改变检测标志
// bool ready = false;
// bool yes_change = false;

int ID;                                         // 补位目标
pps moment;                                     // 实时帧
constraint limit;                               // 约束
vec3d virtual_posi;                             // 虚拟位置
std::vector<vec3d> guide;                       // 指导向量
Guide_vector guider;                            // 封装 指导相邻 & 该向量生成基于的实时帧
bool guide_finish = true;                       // 路径输出完成标志
set3d view_matrix;                              // 测试soket用到
bool isSorted = false;                          // 测试soket用到 用于指示数据是否已排序
bool position_update = true;                    // 位置已更新
std::vector<vec3d> guide_soket;                 // 传输预测轨迹
// CircularQueue* queue = nullptr;                 // 初始化循环队列
CircularQueue queue(300);


void init_target()
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
int main(){
    
    init_target();
    AlgorithmMng am;
    am.start();

    // std::vector<std::vector<set3d>> matrix;              // 初始化时间序列表
    // std::thread app_mavlink(mavlink);

    std::thread timeThread(timegoes, std::ref(moment));
    std::thread loader(loadInCycque, std::ref(moment), std::ref(queue));    // queue.dequeue(first_moment.frame);
    
    // std::thread tube(&AlgorithmMng::start, &am);
    // std::thread consume(consumeInCycque, std::ref(moment), std::ref(queue));                      // 子线程用于剔除旧帧

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));       // 给点时间让ram装载
    // std::thread consume(consumeInCycque, std::ref(queue));
    std::thread planningThread(planning, std::ref(queue), std::ref(ID), std::ref(virtual_posi), std::ref(guider), std::ref(moment), limit, ref(am));      // 输入当前位置 时间 输出期望位置
    std::thread VirtualdroneThread(Virtual_location, std::ref(guider), std::ref(virtual_posi), std::ref(moment), limit);
    
    timeThread.join();
    loader.join();

    // tube.join();
    // consume.join();
    planningThread.join();
    VirtualdroneThread.join();
    
    printf("SUCCESS Finished planning\n");
    // delete queue; queue = nullptr;

    am.stop();

    return 0;
}

