#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <string>
#include <chrono>
#include <condition_variable>
#include <winsock2.h>
#include <ws2tcpip.h>
#include "formation.hpp"
#include "planning.hpp"

// #pragma comment(lib, "Ws2_32.lib")
// #include <chrono>
// #include <cstring>
// #include <sys/socket.h>
// #include <arpa/inet.h>
std::mutex mtx_position;                    // 互斥锁
std::condition_variable cv;                 // 线程信号

int ID;
// vec3d position;
std::vector<vec3d> guide;
pps moment;
constraint limit;
vec3d virtual_posi;
bool guide_finish = false;

void init_target()
{   
    ID = 911;                       // SN from 1 instead of 0
    // position = {9.0, 9.0, 9.0};
    // guide = {9.0, 9.0, 9.0};
    // position = {0.9, 30, 326};
    guide = {0.9, 30, 326};
    guide.resize(0);
    moment = {15};                  // 第95帧开始丢
    // virtual_posi = {9.0, 9.0, 9.0};
    virtual_posi = {0.9, 30, 326};
    // limit.init_position = position;
    // limit.start_frame = moment;

    // limit = constraint(
    //     {0.0, 0.0},      // pps start_frame
    //     {1.0, 2.0, 3.0},   // vec3d init_position
    //     0,                 // int calcu_times
    //     0.0,             // double elapsed_time
    //     6.0,              // double constraint_speed
    //     0.7,               // double collision_radius
    //     ALL_DRONE_NUM,                // int ALL_DRONE_NUM
    //     ACTIVE    // SUCCESS_OR_NOT success
    // );
}

set3d view_matrix;
bool isSorted = false; // 用于指示数据是否已排序

int main() {

    init_target();
    std::vector<std::vector<set3d>> matrix;         // 初始化时间序列表
    Read_frame(matrix);

    std::thread socketThread(socketCommunication);
    std::thread timeThread(timegoes, std::ref(moment));
    std::thread planningThread(planning, matrix, std::ref(ID), std::ref(virtual_posi), std::ref(guide), std::ref(moment), limit);      // 输入当前位置 时间 输出期望位置
    std::thread VirtualdroneThread(Virtual_location, std::ref(guide), std::ref(virtual_posi), std::ref(moment), limit);


    
    socketThread.join();
    timeThread.join();
    planningThread.join();
    VirtualdroneThread.join();

    // auto view_matrix = matrix[75][500];
    printf("Finish planning");

    return 0;
}            