#ifndef __PLANNING__
#define __PLANNING__

#include "AStar.hpp"

void planning(const std::vector<std::vector<set3d>> matrix/*轨迹表*/, int& ID/*丢失的droneID*/, vec3d& position/*当前位置*/, vec3d& output/*输出位置*/, const pps& moment/*时间戳*/, constraint limit/*飞机各类约束*/);
void timegoes(pps& moment);
vec3d simu_position();  

#endif