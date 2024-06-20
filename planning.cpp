#include <thread>
#include <chrono>
#include <cmath>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <mutex>
#include "formation.hpp"
// #include "A_star/AStar.hpp"
#include "AStar.hpp"




// void pilot_phase()
// {
//     // 高度始终跟随补位目标舞步帧的高度
// }

Mint quantizationMapping(const vec3d origin, double rate) {
    Mint result;
    result.x = ((int)origin.x) * (1 / rate) + (int)((origin.x - (int)origin.x) * (1 / rate)) % 10;
    result.y = ((int)origin.y) * (1 / rate) + (int)((origin.y - (int)origin.y) * (1 / rate)) % 10;
    result.z = ((int)origin.z) * (1 / rate) + (int)((origin.z - (int)origin.z) * (1 / rate)) % 10;
    return result;
}
Mint2D quantizationMapping(const vec2d origin, double rate) {
    Mint2D result;
    result.x = ((int)origin.x) * (1 / rate) + (int)((origin.x - (int)origin.x) * (1 / rate)) % 10;
    result.y = ((int)origin.y) * (1 / rate) + (int)((origin.y - (int)origin.y) * (1 / rate)) % 10;
    return result;
}

std::vector<AStar::Vec2i> expandMapping(const double origin, double rate) {
    int result;
    std::vector<AStar::Vec2i> hole;
    AStar::Vec2i block;
    result = ((int)origin) * (1 / rate) + (int)((origin - (int)origin) * (1 / rate)) % 10;
    for (int i = -result; i <= result; i++)
    {
        for (int j = -result; j <= result; j++)
        {
            block.x = i;
            block.y = j;
            hole.push_back(block);
        }
    }
    return hole;
}

vec2d inverMapping(const AStar::Vec2i origin, double rate) {
    vec2d result;
    result.x = origin.x * rate;
    result.y = origin.y * rate;
    return result;
}
Mint2D convertMintToMint2D(const Mint& mint) {
    Mint2D result;
    result.x = mint.x;
    result.y = mint.y;
    return result;
}
#define QUANTIZATION_MAPPING_3D(origin) quantizationMapping(origin, 0.1)   // 0.05的时候取验证一下函数输出
#define QUANTIZATION_MAPPING_2D(origin) quantizationMapping(origin, 0.1)
#define EXPAND_MAPPING_2Dvec(origin) expandMapping(origin, 0.1) 
#define INVERMAPPING(origin) inverMapping(origin, 0.1)                     // 反变换需与变换的映射率一致
#define MintToMin2D(mint) convertMintToMint2D(mint)

bool NEXT = true;
vec3d set3dTovec3d(const set3d& v) {
    return vec3d(v.x, v.y, v.z);
}
#define SET3D_TO_VEC3D(v) set3dTovec3d(v)

vec2d vec3dTovec2d(const vec3d& v) {
    return vec2d(v.x, v.y);
}
#define VEC3D_TO_VEC2D(v) vec3dTovec2d(v)

double euclideanDistance(const vec3d& point1, const vec3d& point2) {
    double dx = pow(point1.x - point2.x, 2);
    double dy = pow(point1.y - point2.y, 2);
    double dz = pow(point1.z - point2.z, 2);
    return sqrt(dx + dy + dz);
}

// AStar::Vec2i vec2dToVec2I(const vec2d& v) {
//     return AStar::Vec2i(v.x, v.y);
// }
// #define VEC2D_TO_VEC2I(v) vec2dToVec2I(v)

AStar::Vec2i Mint2DToVec2I(const Mint2D& v) {
    return AStar::Vec2i(v.x, v.y);
}

std::vector<vec3d> segmentVector(const vec3d& start, const vec3d& end, double l) {              // 长距离分段 l 表示 speed
    
    // auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<vec3d> segments;
    double totalDistance = euclideanDistance(start, end);
    int numSegments = static_cast<int>(std::ceil(totalDistance / l));       // 得到段数
    
    vec3d direction = { (end.x - start.x) / totalDistance,                  // direction 方向向量 斜边满足速度约束 正交边一定满足
                        (end.y - start.y) / totalDistance,
                        (end.z - start.z) / totalDistance };
    
    for (int i = 0/*0包含初始点1不包含*/; i <= numSegments; ++i) {                                // TODO 分段向量超出速度约束的部分 不用再做计算
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

void planning(const std::vector<std::vector<set3d>> matrix/*轨迹表*/, int& ID/*丢失的droneID*/, vec3d& position/*当前位置*/, std::vector<vec3d>& output/*输出位置*/, const pps& moment/*时间戳*/, constraint limit/*飞机各类约束*/)
{   
        // static pps save_moment = {0};   // 初始化静态变量
        // static int receive_same_momenttimes = 0; // 初始化计数器

        // if (save_moment.frame != moment.frame && guide_finish == true) {
        // receive_same_momenttimes = 0;
        // save_moment.frame = moment.frame;}
    extern bool guide_finish;
    guide_finish = false;
    // extern std::mutex mtx_position;
    // mtx_position.lock();
    AStar::Vec2i zero = {0, 0};
    // while (1)       // 配位成功的Flag
    // {   
        // std::unique_lock<std::mutex> lock(mtx_position);
        auto start_time = std::chrono::high_resolution_clock::now(); // 记录开始时间
        if (NEXT)
        {   
            unsigned int frame = moment.frame;                                                                  // 获取当前时间
            set3d target = matrix[frame-1][ID-1];                                                               // 获取补位飞机当前位置
            auto vector_seg = segmentVector(position, SET3D_TO_VEC3D(target), limit.constraint_speed);          // 向量分段 <vec3d> vector_seg (不包含0位置)
            vec3d increment = vector_seg[1] - position;                                                         // 取增量   <vec3d> increment  (往后看一个点)
            Mint guide_target = QUANTIZATION_MAPPING_3D(increment);   // 输入一个vec3d的数据 输出一个实数       // 量化映射 <Mint> x y z
            AStar::Vec2i guide_target_final = Mint2DToVec2I(MintToMin2D(guide_target));
            // step1 起点终点小数已知 √
            // step2 起点终点映射整数(要考虑舞步也要映射成整数 舞步尺度与我的尺度要相同 舞步的全局坐标要转到我0象归一化坐标)
            // step3 生成轨迹解决负方向规划 √
            // step4 整数还原成小数
            // step5 加初向量还原原始轨迹
            // TODO 好像并没有比较高度层


            AStar::Generator generator;                 // 定义了一个generator类
            generator.setWorldSize({200, 200});         // 设置世界地图大小
            // std::vector<Mint2D> wall;                   // 声明墙
            // std::vector<AStar::Vec2i> hole = EXPAND_MAPPING_2Dvec(limit.collision_radius);                      // 设置安全区
            // auto hole_area = hole.size();
            for (size_t i = 0; i < 60; i++)             // todo 向后找多少帧  60帧
            {   
                for (size_t j = 0; j < ALL_DRONE_NUM; j++)      // 检查看看是不是所有飞机都遍历到了
                {
                    vec3d dyschronism = SET3D_TO_VEC3D(matrix[frame-1+i][j]);                                    // 时间上找到障碍帧
                    if (5 > (euclideanDistance(position, dyschronism)))                                          // TODO 欧拉距离改成曼哈顿距离 帧筛选 这里设置规避的障碍半径 这里的 8 应该用速度约束来控
                    // wall.push_back(VEC3D_TO_VEC2D(dyschronism - position));                                   // 这里可以优化的是 不用把方向向量的负球面的那些向量也纳入进来占用遍历时间
                    {//wall.push_back(QUANTIZATION_MAPPING_2D(VEC3D_TO_VEC2D(dyschronism - position)));
                    //  generator.addCollision(VEC2D_TO_VEC2I(QUANTIZATION_MAPPING_2D(VEC3D_TO_VEC2D(dyschronism - position))));
                    // 目标位置不能和障碍是同一个
                    auto Box = Mint2DToVec2I(/*2小数变整Mint2D */QUANTIZATION_MAPPING_2D(/*1直接找出2D障碍小数位置*/VEC3D_TO_VEC2D(dyschronism - position)));   // 与曼哈顿距离合写减少开销
                    // 首尾离得很近的怎么办 去首 去尾
                    if (Box == guide_target_final || Box == zero) continue;        // 屏蔽此刻 始末 位置有飞机占位
                    else {
                        // 这里万一有一个点和0,0离得很近堵死了，答案是不会 因为象限可以四面八方 这是质点的情况 如果扩展成hole那就可能把开始点围起来
                        // 上下左右拓展
                        // std::vector<AStar::Vec2i> expandOfeachbox;
                        // printf("box x:%d y:%d\n",Box.x, Box.y);
                        generator.addCollision(Box);

                            // for (size_t i = 0; i < hole.size(); i++)
                            // {
                            //     generator.addCollision(Box + hole[i]);
                            // }
                        
                    }
                    
                    

                    // 每个位置上做避碰半径拓展，看看能不能把流程放在if后，类型转换前
                    }
                }
            }
          
            generator.setHeuristic(AStar::Heuristic::euclidean);        // 设置启发函数为欧几里得
            generator.setDiagonalMovement(true);                        // 设置对角元素

            std::cout << "Generate path ... \n";
            // auto vewA = Mint2DToVec2I(MintToMin2D(guide_target));
            auto path = generator.findPath({0, 0}, (guide_target_final));               // 库输出路径
            std::reverse(path.begin(), path.end());
            // std::vector<vec3d> path_vec3d;                                           // 对路径反浮点化
            auto max_point = path.size();
            auto perch = increment.z/max_point;
            for (size_t i = 0; i < max_point; i++)
            {   vec3d temp;
                temp.x = position.x + (INVERMAPPING(path[i])).x;
                temp.y = position.y + (INVERMAPPING(path[i])).y;
                temp.z = position.z + perch*i;                                          // 添加对应的 z 补齐成三维
                output.push_back(temp);                     
            }
            
            // 思考未来时刻frame z 上的碰撞
            for(auto& coordinate : output/*path_vec3d*/) {
                std::cout << coordinate.x << " " << coordinate.y << " " << coordinate.z << "\n";
            }

            // 取前面一段 正交在x y 平面
            // if (abs(output.z - target.z)< 0.5){NEXT = 0;}
            // planning_seg_one/*先导段*/(const std::vector<vec3d>);
            // output = vec3d(position.x, position.y, target.z);
            printf("finished planning onetime here!!!!!!!!!!!!!!!!!\n");
            // 接下来虚拟飞机打卡坐标
            // 要更新
            // 线程停止 可以由生成路径的长度 和路径范数球收敛来控制
        }
        else 
        {   
            printf("enter the planning!!!!\n");     // 增加误解情况用上一次的值或原地等待    
        }
        
        // 先导段,先抱持高度同步
        // step3 计算欧氏距离,大致判断到达时间
        // step4 和目标位置的一个向量
        // step5
        // step6 distanceTotarget
        
        // while (1)
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟排序过程的延迟
        // }
        
        // 根据获取的当前时间,和丢失ID,推演跟踪时间
        // 生成全局方位向量
    auto end_time = std::chrono::high_resolution_clock::now(); // 记录结束时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time); // 计算持续时间
    std::cout << "Function execution time: " << duration.count() << " milliseconds" << std::endl;
    // }
    guide_finish = true;

    // mtx_position.unlock();
}






// /*输出轨迹是分段的,是否接入正常舞步,计算延迟(用于约束补偿),当前处于上升段还是规划段*/

// vec3d simu_position()   // 每次肯定是返回一个指导向量
// {
//     while (/* condition */)
//     {
    
    
    
    
//     /* code */
//     }

// }