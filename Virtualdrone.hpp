#ifndef __VIRTUALDRONE__
#define __VIRTUALDRONE__

#include "A_star/AStar.hpp"
#include "formation.hpp"

vec3d Virtual_location(const vec3d& guide, vec3d& virtual_posi, const pps& moment, const constraint limit);    // 虚拟位置 速度乘以指导向量 = 实际位置 速度用的是打卡速度 

#endif