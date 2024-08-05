#ifndef __PLANNING__
#define __PLANNING__

#include "formation.hpp"
#include "AStar.hpp"
#include <mutex>
// #include "algorithmmng.h"

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
        updated = true;
    }
    // 读取函数
    std::pair<std::vector<vec3d>, pps> read() const {
        std::lock_guard<std::mutex> lock(mtx);
        return {guide, moment};
    }
    // 是否已到来新的更新
    bool is_Update_or_not(){
        if (updated = true) 
        {updated = false;
        return true;}
        
        else return false;
    }

private:
    mutable std::mutex mtx; // 保护更新操作的互斥锁
    bool updated = false; // 是否已更新的标志
};


// void receive(AlgorithmMng& am);

// void timegoes(pps& moment);
// void Virtual_location(const Guide_vector&, vec3d& virtual_posi, const pps& moment, const constraint limit);    // 虚拟位置 速度乘以指导向量 = 实际位置 速度用的是打卡速度 
// void monitor(const vec3d& virtual_posi);
// vec3d simu_position();  

#endif