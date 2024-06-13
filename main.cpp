#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include "formation.hpp"
#include "planning.hpp"

// #include <chrono>
// #include <cstring>
// #include <sys/socket.h>
// #include <arpa/inet.h>
std::vector<std::vector<set3d>> matrix;         // 初始化时间序列表


int main() {
    Read_frame(matrix);
    std::thread planningThread(planning);
    // std::thread readThread(Read_frame); 
    // std::thread socketThread(socketCommunication);
    // readThread.join();
    // socketThread.join();


    // auto view_matrix = matrix[75][500];
    printf("Finish planning");

    return 0;
}            