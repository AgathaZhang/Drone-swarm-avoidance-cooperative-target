#ifndef __PLANNING__
#define __PLANNING__

#include "AStar.hpp"
#include <mutex>

class Guide_vector {
public:
    std::vector<vec3d>& guide;
    pps moment;

    // 构造函数
    Guide_vector(std::vector<vec3d>& guide = *(new std::vector<vec3d>()), const pps& moment = pps())
        : guide(guide), moment(moment) {}

    // 更新函数
    void update(std::vector<vec3d>& new_guide, const pps& new_moment) {
        std::lock_guard<std::mutex> lock(mtx);
        guide = new_guide;
        moment = new_moment;
    }

private:
    std::mutex mtx; // 保护更新操作的互斥锁
};



void planning(const std::vector<std::vector<set3d>> matrix/*轨迹表*/, int& ID/*丢失的droneID*/,const vec3d& position/*当前位置*/, std::vector<vec3d>& output/*输出位置*/, const pps& moment/*时间戳*/, constraint limit/*飞机各类约束*/);
void timegoes(pps& moment);
void Virtual_location(const std::vector<vec3d>& guide, vec3d& virtual_posi, const pps& moment, const constraint limit);    // 虚拟位置 速度乘以指导向量 = 实际位置 速度用的是打卡速度 
vec3d simu_position();  

#endif