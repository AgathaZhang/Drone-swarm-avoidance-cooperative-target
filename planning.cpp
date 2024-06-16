#include <thread>
#include <chrono>
#include <cmath>
#include <chrono>
#include <iostream>
#include "formation.hpp"
#include "A_star/AStar.hpp"

void timegoes(pps& moment) {
    while (1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        moment.frame ++;
        printf("moment now :%d\n", moment.frame);
    }
}

// void pilot_phase()
// {
//     // 高度始终跟随补位目标舞步帧的高度
// }
bool guide = true;
vec3d set3dTovec3d(const set3d& v) {
    return vec3d(v.x, v.y, v.z);
}
#define SET3D_TO_VEC3D(v) set3dTovec3d(v)

double euclideanDistance(const vec3d& point1, const vec3d& point2) {
    double dx = pow(point1.x - point2.x, 2);
    double dy = pow(point1.y - point2.y, 2);
    double dz = pow(point1.z - point2.z, 2);
    return sqrt(dx + dy + dz);
}

std::vector<vec3d> segmentVector(const vec3d& start, const vec3d& end, double l) {              // 长距离分段
    
    // auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<vec3d> segments;
    double totalDistance = euclideanDistance(start, end);
    int numSegments = static_cast<int>(std::ceil(totalDistance / l));       // 得到段数
    
    vec3d direction = { (end.x - start.x) / totalDistance,                  // 斜边满足速度约束 正交边一定满足
                        (end.y - start.y) / totalDistance,
                        (end.z - start.z) / totalDistance };
    
    for (int i = 0; i <= numSegments; ++i) {
        vec3d segment = { start.x + direction.x * l * i,
                          start.y + direction.y * l * i,
                          start.z + direction.z * l * i };
        segments.push_back(segment);
    }

    // Ensure the end point is included
    if (euclideanDistance(segments.back(), end) > 1e-6) { // Avoid floating point comparison issues
        segments.push_back(end);
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::micro> duration = end_time - start_time;
    // std::cout << "Time taken for segmentVector comparisons: " << duration.count() << " microseconds" << std::endl;
    return segments;
}

// vec3d planning_seg_one/*先导段*/(vec3d position/*当前位置*/, pps moment/*时间戳*/) {
//     // step1 原地升高到切片高度
//     // step2 

// }

void planning(const std::vector<std::vector<set3d>> matrix/*轨迹表*/, int& ID/*丢失的droneID*/, vec3d& position/*当前位置*/, vec3d& output/*输出位置*/, const pps& moment/*时间戳*/, constraint limit/*飞机各类约束*/)
{   
    while (1)       // 配位成功的Flag
    {    
        if (guide)
        {   
            unsigned int frame = moment.frame;
            set3d target = matrix[frame-1][ID-1];             // step1 获取补位飞机当前位置
            auto vector_seg = segmentVector(position, SET3D_TO_VEC3D(target), limit.constraint_speed);        // 向量分段


            // planning_seg_one/*先导段*/(const std::vector<vec3d>);
        }
        else 
        {
            
        }
        
        // 先导段,先抱持高度同步
        // step3 计算欧氏距离,大致判断到达时间
        // step4 和目标位置的一个向量
        // step5
        // step6 distanceTotarget
        
        while (1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟排序过程的延迟
        }
        
        // 根据获取的当前时间,和丢失ID,推演跟踪时间
        // 生成全局方位向量
    }
}






// /*输出轨迹是分段的,是否接入正常舞步,计算延迟(用于约束补偿),当前处于上升段还是规划段*/

// vec3d simu_position()   // 每次肯定是返回一个指导向量
// {
//     while (/* condition */)
//     {
    
    
    
    
//     /* code */
//     }

// }