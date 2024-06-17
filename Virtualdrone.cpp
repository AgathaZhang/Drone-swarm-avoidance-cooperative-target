#include <thread>
#include <chrono>
#include <cmath>
#include <chrono>
#include <iostream>
#include "formation.hpp"
#include "AStar.hpp"

void timegoes(pps& moment) {
    while (1)
    {   extern vec3d output;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        moment.frame ++;
        printf("moment now :%d\n", moment.frame);
        // printf("guide_vec now :x %f\n, y %f\n, z %f\n", output.x, output.x, output.z);
    }
}

vec3d Virtual_location(const vec3d& guide, vec3d& virtual_posi, const pps& moment, const constraint limit) {
    vec3d A;
    return A;
}     // 虚拟位置 速度乘以指导向量 = 实际位置 速度用的是打卡速度 