#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include "formation.hpp"
// #include "planning.hpp"
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <winsock2.h>
#include <ws2tcpip.h>
#include "formation.hpp"
// #pragma comment(lib, "Ws2_32.lib")
// #include <chrono>
// #include <cstring>
// #include <sys/socket.h>
// #include <arpa/inet.h>
// std::vector<std::vector<set3d>> matrix;         // 初始化时间序列表
set3d view_matrix;
bool isSorted = false; // 用于指示数据是否已排序
int main() {

    //   // 初始化 Winsock 库
    // WSADATA wsaData;
    // int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    // if (result != 0) {
    //     std::cerr << "WSAStartup failed" << std::endl;
    //     return 1;
    // }
 
    // // 在这里使用 Winsock 的函数进行网络编程
 
    // // 关闭 Winsock 库
    // WSACleanup();

    std::vector<std::vector<set3d>> matrix;         // 初始化时间序列表
    Read_frame(matrix);
    // std::thread planningThread(planning);
    // std::thread readThread(Read_frame); 
    // readThread.join();
    std::thread socketThread(socketCommunication);
    for (size_t i = 0; i < 99; i++)
    {   for (size_t j = 0; j < 1000; j++)
            {
                view_matrix = matrix[i][j];
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟排序过程的延迟
                // printf("series frame: %d, drone: %d\n", i, j);
            }
    }
    isSorted = true; // 标记排序完成
    extern void socketCommunication(void);

    socketThread.join();
    

    // auto view_matrix = matrix[75][500];
    printf("Finish planning");

    return 0;
}            