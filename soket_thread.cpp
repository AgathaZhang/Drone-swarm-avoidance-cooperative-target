#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <socket.h>
#include <arpa/inet.h>
#include <unistd.h>

void socketCommunication() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // 创建套接字文件描述符
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // 附加套接字到端口
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8080);

    // 绑定套接字到端口
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // 监听端口
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    // 接受客户端连接
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    while (true) {
        std::lock_guard<std::mutex> lock(dataMutex);
        std::string status = "Current array: ";
        for (int num : data) {
            status += std::to_string(num) + " ";
        }
        status += isSorted ? "Sorted" : "Sorting";
        send(new_socket, status.c_str(), status.size(), 0);
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 每秒发送一次
        if (isSorted) break;
    }

    close(new_socket);
    close(server_fd);
}