#ifndef __PLANNING__
#define __PLANNING__

#include "AStar.hpp"

void planning(const std::vector<std::vector<set3d>> matrix/*轨迹表*/, int& ID/*丢失的droneID*/, vec3d& position/*当前位置*/, std::vector<vec3d>& output/*输出位置*/, const pps& moment/*时间戳*/, constraint limit/*飞机各类约束*/);
void timegoes(pps& moment);
void Virtual_location(const std::vector<vec3d>& guide, vec3d& virtual_posi, const pps& moment, const constraint limit);    // 虚拟位置 速度乘以指导向量 = 实际位置 速度用的是打卡速度 
vec3d simu_position();  

#endif