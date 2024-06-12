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

std::vector<int> data = {5, 3, 8, 6, 2, 7, 4, 1};
std::mutex dataMutex;
bool isSorted = false;

void bubbleSort() {
    size_t n = data.size();
    for (size_t i = 0; i < n - 1; ++i) {
        for (size_t j = 0; j < n - i - 1; ++j) {
            std::lock_guard<std::mutex> lock(dataMutex);
            if (data[j] > data[j + 1]) {
                std::swap(data[j], data[j + 1]);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟排序过程的延迟
    }
    std::lock_guard<std::mutex> lock(dataMutex);
    isSorted = true;
}