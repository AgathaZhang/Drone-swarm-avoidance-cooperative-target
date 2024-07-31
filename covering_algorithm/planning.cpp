#include <thread>
#include <chrono>
#include <cmath>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <mutex>
#include <condition_variable>
#include "formation.hpp"
#include "planning.hpp"
// #include "A_star/AStar.hpp"
#include "AStar.hpp"
#include "algorithmmng.h"




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

// std::vector<AStar::Vec2i> expandMapping(const double origin, double rate) {
//     int result;
//     std::vector<AStar::Vec2i> hole;
//     AStar::Vec2i block;
//     result = ((int)origin) * (1 / rate) + (int)((origin - (int)origin) * (1 / rate)) % 10;
//     for (int i = -result; i <= result; i++)
//     {
//         for (int j = -result; j <= result; j++)
//         {
//             block.x = i;
//             block.y = j;
//             hole.push_back(block);
//         }
//     }
//     return hole;
// }
std::vector<AStar::Vec3i> expandMapping(const double origin, double rate) {
    int result;
    std::vector<AStar::Vec3i> hole;
    AStar::Vec3i block;
    result = ((int)origin) * (1 / rate) + (int)((origin - (int)origin) * (1 / rate)) % 10;
    for (int i = -result; i <= result; i++) {
        for (int j = -result; j <= result; j++) {
            for (int k = -result; k <= result; k++) {
                block.x = i;
                block.y = j;
                block.z = k;
                hole.push_back(block);
            }
        }
    }
    return hole;
}
vec3d inverMapping(const AStar::Vec3i origin, double rate) {
    vec3d result;
    result.x = origin.x * rate;
    result.y = origin.y * rate;
    result.z = origin.z * rate;
    return result;
}

Mint2D convertMintToMint2D(const Mint& mint) {
    Mint2D result;
    result.x = mint.x;
    result.y = mint.y;
    return result;
}
#define QUANTIZATION_MAPPING_3D(origin) quantizationMapping(origin, 0.1)    // 0.05的时候取验证一下函数输出
#define QUANTIZATION_MAPPING_2D(origin) quantizationMapping(origin, 0.1)
#define EXPAND_MAPPING_2Dvec(origin) expandMapping(origin, 0.1)             // 质点体积扩增
#define INVERMAPPING(origin) inverMapping(origin, 0.1)                      // 反变换需与变换的映射率一致
#define MintToMin2D(mint) convertMintToMint2D(mint)


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

std::pair<double, vec3d> manhattanDistance(const vec3d& point1/*本体飞机*/, const vec3d& point2/*障碍飞机*/) {
    double dx = point2.x - point1.x;
    double dy = point2.y - point1.y;
    double dz = point2.z - point1.z;
    return std::make_pair(std::fabs(dx) + std::fabs(dy) + std::fabs(dz), vec3d{dx, dy, dz});
}
// AStar::Vec2i vec2dToVec2I(const vec2d& v) {
//     return AStar::Vec2i(v.x, v.y);
// }
// #define VEC2D_TO_VEC2I(v) vec2dToVec2I(v)

AStar::Vec3i Mint3DToVec3I(const Mint& v) {
    return AStar::Vec3i(v.x, v.y, v.z);
}

std::vector<vec3d> segmentVector(const vec3d& start, const vec3d& end, double l) {              // 长距离分段 l 表示 speed
    
    // auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<vec3d> segments;
    double totalDistance = euclideanDistance(start, end);                   // 因为涉及到速度约束 这里似乎只能用欧拉距离了
    int numSegments = static_cast<int>(std::ceil(totalDistance / l));       // 得到段数
    
    vec3d direction = { (end.x - start.x) / totalDistance,                  // direction 方向向量 斜边满足速度约束 正交边一定满足
                        (end.y - start.y) / totalDistance,
                        (end.z - start.z) / totalDistance };
    
    for (int i = 0/*0包含初始点1不包含*/; i <= 3/*numSegments*/; ++i) {                                // TODO 分段向量超出速度约束的部分 不用再做计算 目前只取 3 段
        vec3d segment = { start.x + direction.x * l * i,
                          start.y + direction.y * l * i,
                          start.z + direction.z * l * i };
        segments.push_back(segment);
    }

    // Ensure the end point is included 只取前三段 所以不用末端处理
        // if (euclideanDistance(segments.back(), end) > 1e-6) { // Avoid floating point comparison issues
        //     segments.push_back(end);
        // }
    // auto end_time = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::micro> duration = end_time - start_time;
    // std::cout << "Time taken for segmentVector comparisons: " << duration.count() << " microseconds" << std::endl;
    return segments;
}

// vec3d planning_seg_one/*先导段*/(vec3d position/*当前位置*/, pps moment/*时间戳*/) {
//     // step1 原地升高到切片高度
//     // step2 

// }

bool NEXT = true;
void planning(CircularQueue& queue/*轨迹表*/, int& ID/*丢失的droneID*/,const vec3d& origin_position/*当前位置*/, Guide_vector& guider/*输出指导向量*/, const pps& origin_moment/*时间戳*/, constraint limit/*飞机各类约束*/, AlgorithmMng &am)
{   
    mavlink_auto_filling_dance_t singleSend_msg;        // 定义每次发送给飞机的期望向量
    // matrix 在这里要重新定义一下 以弹出校验的形式(启用)
    // extern bool parameter_changed;
    // extern std::mutex changed; 
    // extern std::condition_variable cv;
    queue.dequeue(origin_moment.frame);     // 内部消耗更合理
    extern std::mutex mtx_position;
    // extern bool yes_change;
    extern bool guide_finish;
    // AStar::Vec2i zero = {0, 0};         // 复用: 1 起始点 2 质点扩增初始化
    AStar::Vec3i zero = {0, 0, 0};
    while (guide_finish == true)        // 计算完成 || 超时 || 无解 其他情况丢给异常处理线程 如果位置没移动，那么线程挂起
    {   
        guide_finish = false;           // 清空标志位
        mtx_position.lock();            // 这里似乎没有必要加锁
        // std::unique_lock<std::mutex> lock(mtx_position);
        const vec3d position = origin_position;         // 获取 position
        mtx_position.unlock();
        const pps moment = origin_moment;               // 获取 moment 本轮while循环中const pps moment不再改变 直到下轮循环
        // auto start_time = std::chrono::high_resolution_clock::now();                                            // 记录开始时间用于测算单次路径规划的耗时
        // std::unique_lock<std::mutex> lock(changed);
        // cv.wait(lock, []{ return parameter_changed; });
        // std::this_thread::sleep_for(std::chrono::milliseconds(43)); 
        if (NEXT)       // 正常完成计算时进入 否则进入else
        {   
            unsigned int frame = moment.frame;                                                                  // 获取当前帧
            // if(!queue.dequeue(matrix))printf("current sequence pop go wrong\n");
            queue.atomicity = 0;                                                                                // 锁定cycbuffer的原子时间，期间不可dequeue以确保算法单次规划是一致可微的
            singleSend_msg.frame = frame;                                                                       // TODO 这里基于的时间需要有原子性吗？
            set3d target = queue.invoking(frame, (ID-1));                                                       // 获取目标当前位置
            // set3d target = matrix[frame-1][ID-1];                                                            // 获取目标当前位置
            auto vector_seg = segmentVector(position, SET3D_TO_VEC3D(target), limit.constraint_speed);          // 向量分段 <vec3d> vector_seg (不包含0位置)
            vec3d increment = vector_seg[1] - position;                                                         // 取增量   <vec3d> increment  (往后看一个点)
            Mint guide_target = QUANTIZATION_MAPPING_3D(increment);                                             // 输入一个 vec3d的数据 量化映射到 <Mint> x y z
            AStar::Vec3i guide_target_final = Mint3DToVec3I(guide_target);                                        // 格式转换
            // printf("target : %d, %d, %d\n", guide_target_final.x,guide_target_final.y,guide_target_final.z);
            // AStar::Vec2i guide_target_final = Mint2DToVec2I(MintToMin2D(guide_target));                         // 2舍维到 2d 转 2i
            // step1 起点终点小数已知 √
            // step2 起点终点映射整数(要考虑舞步也要映射成整数 舞步尺度与我的尺度要相同 舞步的全局坐标要转到我0象归一化坐标)
            // step3 生成轨迹解决负方向规划 √
            // step4 整数还原成小数
            // step5 加初向量还原原始轨迹
            // TODO 好像并没有比较高度层


            AStar::Generator generator;                                                                             // 定义了一个generator类
            generator.setWorldSize({100, 100, 100});                                                                // 设置世界地图大小
            // std::vector<Mint2D> wall;                                                                            // 声明墙
            std::vector<AStar::Vec3i> hole = EXPAND_MAPPING_2Dvec(limit.collision_radius);                          // 设置安全区hole
            // if (queue.buffer_count_() < 60){std::this_thread::sleep_for(std::chrono::milliseconds(10));}                                        // 睡眠等待填满
            for (size_t i = 0; i < 60; i++)             // TODO 向后找多少帧 60帧 这里根据速度约束在单次计算的平均时间开销来推断,尽量的小,避免时序上过长 wall堵塞造成无解的情况
            {   
                for (size_t j = 0; j < ALL_DRONE_NUM; j++)      // 检查看看是不是所有飞机都遍历到了
                {
                    // vec3d dyschronism = SET3D_TO_VEC3D(matrix[frame-1+i][j]);                                    // 时间上找到障碍帧
                    vec3d dyschronism = SET3D_TO_VEC3D(queue.invoking(frame, (ID-1)));
                    auto range = manhattanDistance(position, dyschronism).first;    // 返回 x+y+z
                    auto spot = manhattanDistance(position, dyschronism).second;    // 返回 差diff vec3d xyz
                    if (8/*这里选取障碍范围*/ > range)                                          // 找当前位置相邻范围 TODO 06.21待讨论 思考：用曼哈顿距离 后 会不会引入更多的 非同层点的投影 以此影响有解的可能性 曼哈顿距离和欧拉距离的适用场景 欧拉距离改成曼哈顿距离 帧筛选 这里设置规避的障碍半径 这里的 8 应该用速度约束来控
                    // wall.push_back(VEC3D_TO_VEC2D(dyschronism - position));                                   // 这里可以优化的是 不用把方向向量的负球面的那些向量也纳入进来占用遍历时间
                    //wall.push_back(QUANTIZATION_MAPPING_2D(VEC3D_TO_VEC2D(dyschronism - position)));
                    //  generator.addCollision(VEC2D_TO_VEC2I(QUANTIZATION_MAPPING_2D(VEC3D_TO_VEC2D(dyschronism - position))));
                    // 目标位置不能和障碍是同一个
                    {
                    auto Box = Mint3DToVec3I/*格式转换*/(QUANTIZATION_MAPPING_3D(spot));
                    // auto Box = Mint2DToVec2I(/*2小数变整Mint2D */QUANTIZATION_MAPPING_2D(/*1直接找出2D障碍小数位置*/VEC3D_TO_VEC2D(dyschronism - position)));   // √ 与曼哈顿距离合写减少开销
                    // 首尾离得很近的怎么办 去首 去尾
                        // if (Box == guide_target_final || Box == zero) continue;                                      // 屏蔽此刻 始末 位置有飞机占位
                        // else generator.addCollision(Box);
                    for (size_t i = 0; i < hole.size(); i++)
                    {
                        generator.addCollision(Box + hole[i]);              //TODO 06.28 考虑包围进0,0点堵死的情况
                    }                                                       //TODO 07.22 同上 考虑离飞机很近的hole面积，排除掉
                    // generator.addCollision(Box);
                    }
                    // auto hole_area = hole.size();
                        // 这里万一有一个点和0,0离得很近堵死了，答案是不会 因为象限可以四面八方 这是质点的情况 如果扩展成hole那就可能把开始点围起来
                        // 上下左右拓展
                        // std::vector<AStar::Vec2i> expandOfeachbox;
                        // printf("box x:%d y:%d\n",Box.x, Box.y);
                        


                        
                    
                    
                    

                    // 每个位置上做避碰半径拓展，看看能不能把流程放在if后，类型转换前
                    
                }
            }
            queue.atomicity = 1;                                                                         // 释放cycbuffer的原子时间
          
            generator.setHeuristic(AStar::Heuristic::euclidean);        // 设置启发函数为欧几里得
            generator.setDiagonalMovement(true);                        // 设置对角元素

            // // std::cout << "Generate path ... \n";
            // // auto vewA = Mint2DToVec2I(MintToMin2D(guide_target));
            auto path = generator.findPath({0, 0}, (guide_target_final));               // 库输出路径
            std::reverse(path.begin(), path.end());                                     // 反向vector还可以继续优化计算开销

            std::vector<vec3d> output;                                           
            auto max_point = path.size();
            // auto perch = increment.z/max_point;
            {   
                for (size_t i = 0; i < max_point; i++)
                {   vec3d temp;
                    temp.x = position.x + (INVERMAPPING(path[i])).x;                        // 对路径反浮点化
                    temp.y = position.y + (INVERMAPPING(path[i])).y;
                    temp.z = position.z + (INVERMAPPING(path[i])).z;
                    // temp.z = position.z + perch*i;                                          // 添加对应的 z 补齐成三维
                    output.push_back(temp);                                                 // 思考未来时刻frame z 上的碰撞
                }
                guider.update(output, frame);
                {
                /** 发送的业务*/
                singleSend_msg.x = static_cast<float> (output[1].x);
                singleSend_msg.y = static_cast<float> (output[1].y);
                singleSend_msg.z = static_cast<float> (output[1].z);
                am.send_planningPosition(&singleSend_msg);
                }  

                // mtx_position.unlock();
            }   // `lock` 在这里作用域结束自动解锁
                                                                        
            // for(auto& coordinate : output/*path_vec3d*/) {
            //     std::cout << coordinate.x << " " << coordinate.y << " " << coordinate.z << "\n";
            // }

            // 取前面一段 正交在x y 平面
            // if (abs(output.z - target.z)< 0.5){NEXT = 0;}
            // planning_seg_one/*先导段*/(const std::vector<vec3d>);
            // output = vec3d(position.x, position.y, target.z);
            // printf("finished planning onetime here!!!!!!!!!!!!!!!!!\n");
            // 接下来虚拟飞机打卡坐标
            // 要更新
            // 线程停止 可以由生成路径的长度 和路径范数球收敛来控制
        }
        else 
        {   
            // printf("enter the planning!!!!\n");     // 增加误解情况用上一次的值或原地等待    
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
    
    // auto end_time = std::chrono::high_resolution_clock::now(); // 记录结束时间
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time); // 计算持续时间
    // std::cout << "Function execution time: " << duration.count() << " milliseconds" << std::endl;
    guide_finish = true; 
    }
           // 置计算成功标志位
}






// /*输出轨迹是分段的,是否接入正常舞步,计算延迟(用于约束补偿),当前处于上升段还是规划段*/

// vec3d simu_position()   // 每次肯定是返回一个指导向量
// {
//     while (/* condition */)
//     {
    
    
    
    
//     /* code */
//     }

// }