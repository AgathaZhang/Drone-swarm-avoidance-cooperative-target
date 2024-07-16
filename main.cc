#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <string>
#include <chrono>
#include <condition_variable>
// #include <winsock2.h>
// #include <ws2tcpip.h>             // && 临时注释
// #include <sys/socket.h>           // && 临时注释
// #include <arpa/inet.h>
#include "planning/formation.hpp"
// #include "planning.hpp"
// #pragma comment(lib, "Ws2_32.lib")

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
// Guide_vector guider;                            // 封装 指导相邻 & 该向量生成基于的实时帧 && 临时注释
bool guide_finish = true;                       // 路径输出完成标志
set3d view_matrix;                              // 测试soket用到
bool isSorted = false;                          // 测试soket用到 用于指示数据是否已排序
bool position_update = true;                    // 位置已更新
std::vector<vec3d> guide_soket;                 // 传输预测轨迹

void init_target()
{   
    // ID = 911;                       // SN from 1 instead of 0
    ID = 5;
    moment = {10};                  // 第95帧开始丢
    virtual_posi = {0, 0, 0};
    // virtual_posi = {7, 23, 335};
    // virtual_posi = {-160, 70, 150};
    // printf("SUCCESS init\n");
}
int main(){
    
    init_target();

    // std::vector<std::vector<set3d>> matrix;         // 初始化时间序列表
    // Read_frame(matrix);                             //&& 这里需要根据当前帧选择 临时Read_frame的sdcard区域
    Read_frame(0/*帧序*/);
    printf("SUCCESS Finished planning\n");
    return 0;
}
