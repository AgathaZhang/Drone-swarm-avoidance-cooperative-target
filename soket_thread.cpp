#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <winsock2.h>
#include <ws2tcpip.h>
#include "formation.hpp"
#pragma comment(lib, "Ws2_32.lib")

void socketCommunication() {
    // extern bool isSorted;           // 声明
    // extern set3d view_matrix;       // 声明
    WSADATA wsaData;
    int iResult;

    // 初始化 WinSock 库
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        std::cerr << "WSAStartup failed with error: " << iResult << std::endl;
        return;
    }



    SOCKET server_fd = INVALID_SOCKET;
    SOCKET new_socket = INVALID_SOCKET;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);



    // 创建套接字文件描述符
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == INVALID_SOCKET) {
        std::cerr << "Socket failed with error: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return;
    }

    // 附加套接字到端口
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) == SOCKET_ERROR) {
        std::cerr << "setsockopt failed with error: " << WSAGetLastError() << std::endl;
        closesocket(server_fd);
        WSACleanup();
        return;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(115);

    // 绑定套接字到端口
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) == SOCKET_ERROR) {
        std::cerr << "Bind failed with error: " << WSAGetLastError() << std::endl;
        closesocket(server_fd);
        WSACleanup();
        return;
    }

    // 监听端口
    if (listen(server_fd, 3) == SOCKET_ERROR) {
        std::cerr << "Listen failed with error: " << WSAGetLastError() << std::endl;
        closesocket(server_fd);
        WSACleanup();
        return;
    }

    // 接受客户端连接
    new_socket = accept(server_fd, (struct sockaddr *)&address, &addrlen);
    if (new_socket == INVALID_SOCKET) {
        std::cerr << "Accept failed with error: " << WSAGetLastError() << std::endl;
        closesocket(server_fd);
        WSACleanup();
        return;
    }

    while (true) {
        // std::lock_guard<std::mutex> lock(dataMutex); // 使用互斥锁保护共享数据
        std::string status = "Current view_matrix: ";
        status += std::to_string(view_matrix.x) + " " + std::to_string(view_matrix.y) + " " + std::to_string(view_matrix.z);
        status += isSorted ? " Sorted" : " Sorting";
        send(new_socket, status.c_str(), status.size(), 0); // 发送状态信息到客户端
        // std::this_thread::sleep_for(std::chrono::seconds(2)); // 每秒发送一次
        // if (isSorted) break;
    }

    closesocket(new_socket); // 关闭客户端套接字
    closesocket(server_fd); // 关闭服务器套接字
    WSACleanup(); // 清理 WinSock 库
}