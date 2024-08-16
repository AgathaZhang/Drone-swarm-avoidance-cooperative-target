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
#include "AStar.hpp"
#include "algorithmmng.h"

namespace std {     // 在std::hash空间中特化对AStar::Vec3i类的哈希运算

    template <>
    struct hash<AStar::Vec3i> { // 重载 () 操作符，用于生成基于 Vec3i 对象的哈希值
        size_t operator()(const AStar::Vec3i& v) const {
            return std::hash<int>()(v.x) ^ std::hash<int>()(v.y) ^ std::hash<int>()(v.z);
        }
    };

    // 为 AStar::Vec3i 类型自定义 std::equal_to 特化版本
    template <>
    struct equal_to<AStar::Vec3i> { // 重载()，用于比较两个 Vec3i 对象是否相等
        bool operator()(const AStar::Vec3i& lhs, const AStar::Vec3i& rhs) const {
            return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z; 
// 注意在哈希比较两个 Vec3i 对象的每个坐标是否相等时必须先整型量化 实型只能考虑范数球 实型不是离散的 不能比较哈希相等
        }
    };
}

void RGB_control(mavlink_auto_filling_dance_t& singleSend_msg, int phase, set3d target = {0}){

    switch (phase)
    {
    case 1:     /** 白光*/
        singleSend_msg.rgb[0] = 255;
            singleSend_msg.rgb[1] = 255;
                singleSend_msg.rgb[2] = 255;
    break;
    
    case 2:     /** 红光*/
        singleSend_msg.rgb[0] = 255;
            singleSend_msg.rgb[1] = 0;
                singleSend_msg.rgb[2] = 0;
    break;
    
    case 3:     /** 绿光*/
        singleSend_msg.rgb[0] = target.R;
            singleSend_msg.rgb[1] = target.G;
                singleSend_msg.rgb[2] = target.B;
    break;

    }

}

void AlgorithmMng::exception_handling(){
    int casenum = 0;
    if(termination = 1){casenum = 1;}               // 终止补位
    if(bad_quadrantDrone_num > 5){casenum = 2;}     // 连续倒退
    if(solution_time > 1){casenum = 3;}             // 解超时
    if(inversePlanning = 1){casenum = 4;}           // 解反向
    if(cycbuffer_residue < 60){casenum = 5;}        // cycbuffer 容量告急

    switch (casenum)
    {
    case 1:
        /* code */
        break;
    
    default:
        break;
    }
}

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

std::vector<AStar::Vec3i> expandMapping(const double origin, double rate) {         // 可以设置椭球率 因为飞机实际上是扁的
    int result = static_cast<int>(origin * (1 / rate));     // 计算半径
    // int result = ((int)origin) * (1 / rate) + (int)((origin - (int)origin) * (1 / rate)) % 10;
    std::vector<AStar::Vec3i> hole;
    AStar::Vec3i block;

    for (int i = -result; i <= result; i++) {
        for (int j = -result; j <= result; j++) {
            for (int k = -result; k <= result; k++) {
                if (i * i + j * j + k * k <= result * result) {
                    block.x = i;
                    block.y = j;
                    block.z = k;
                    hole.push_back(block);
                }
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
#define QUANTIZATION_MAPPING_3D(origin) quantizationMapping(origin, 0.1)    
#define QUANTIZATION_MAPPING_2D(origin) quantizationMapping(origin, 0.1)    // 0.05的时候取验证一下函数输出
#define EXPAND_MAPPING_3Dvec(origin) expandMapping(origin, 0.1)             // 质点体积扩增
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

AStar::Vec3i Mint3DToVec3I(const Mint& v) {
    return AStar::Vec3i(v.x, v.y, v.z);
}

std::vector<vec3d> segmentVector(const vec3d& start, const vec3d& end, double l, double& endpoint_distance) {              // 长距离分段 l 表示 speed
    
    // auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<vec3d> segments;
    double totalDistance = euclideanDistance(start, end);                   // 因为涉及到速度约束 这里似乎只能用欧拉距离了
    endpoint_distance = totalDistance;
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

void AlgorithmMng::Pos_estimator(void){

   pos_predict.x = virtual_posi.x + (velocity.x * solution_time);
   pos_predict.y = virtual_posi.y + (velocity.y * solution_time);
   pos_predict.z = virtual_posi.z + (velocity.z * solution_time);
    
//    pos_predict.x = virtual_posi.x + (velocity.x * (solution_time + sleep_seconds));       // 速度 * 0.5s后的时间 = 预测位置 (这么做肯定不精准)
//    pos_predict.y = virtual_posi.y + (velocity.y * (solution_time + sleep_seconds));
//    pos_predict.z = virtual_posi.z + (velocity.z * (solution_time + sleep_seconds));
   
}

/**  分为先导段 避障段 末端段 */
void AlgorithmMng::planning(CircularQueue& queue/*轨迹表*/, int& ID/*丢失的droneID*/, const vec3d& origin_position/*当前位置*//*, Guide_vector& sub_guider/*输出指导向量*/, const pps& origin_moment/*时间戳*/, constraint limit/*飞机各类约束*/)
{   
    // std::chrono::milliseconds duration(sleep_time);     // 用于速度积分初始化 将 duration 转换为秒
    // std::chrono::duration<double> duration_in_seconds = duration;
    // sleep_seconds = duration_in_seconds.count();       // 用于速度积分初始化获取秒数

    // bool enter_endingpoint = false;     // 强制进入endingpoint测试用
    // static int cont_planning = 0;       // 强制进入endingpoint测试用
    const auto frame_duration = std::chrono::milliseconds(1000 / danceFrame_rate);          // 用于控制帧速率间隔长度
    auto start_1 = std::chrono::high_resolution_clock::now();                               // 先导段计时器
    auto next_frame = std::chrono::steady_clock::now();                                     // 先导段和末端段计时器
    mavlink_auto_filling_dance_t singleSend_msg;                                            // 定义每次发送给飞机的mavlink
    
    double R_manhattanball = 1.72 * limit.constraint_speed;                                 // R_manhattanball 用于设置寻找局部规划中所有的障碍,曼哈顿球要内接欧拉球需要扩大根号3倍半径
    AStar::Vec3i zero = {0, 0, 0};                                                          // 定义质点体积补偿hole 复用: 1 起始点 2 质点扩增初始化 // (二维)AStar::Vec2i zero = {0, 0};
    std::vector<AStar::Vec3i> hole = EXPAND_MAPPING_3Dvec(limit.collision_radius);          // 设置安全区hole

    while (termination == false)        // 计算完成 || 超时 || 无解 其他情况丢给异常处理线程 如果位置没移动，那么线程挂起
    {    
        mtx_position.lock();
        Pos_estimator();                                                                    // 预测更新
        // const vec3d position = origin_position;                                             // 获取 position 赋值给const 本轮while循环中const pps moment不再改变 直到下轮循环
        const vec3d position = pos_predict;                                                 // 预测位置
        const pps moment = origin_moment;                                                   // 获取 moment      
        mtx_position.unlock();
        singleSend_msg.frame = moment.frame;
        
        // while (true){
        //     int monitor_counter = 0;
        //     if(queue.buffer_count_() > 60)  {                                                       // 余量监控 防止耗尽保证安全
        //         queue.dequeue(moment.frame);                                                        // 当前已刷新到实时 更新cycbuffer 
        //         break;                      }     

            
        //     else if(queue.buffer_count_() < 60 && monitor_counter > 5)                                                 // 舞步快完了 跳过监控
        //     {
        //         printf("queue.buffer_count_() < 60 && monitor_counter > 5\n");
        //     }                                                                           
        //     else{
        //         std::this_thread::sleep_for(std::chrono::milliseconds(1));                          // 等待装填
        //         monitor_counter++;
        //         if (monitor_counter >= 6) {printf("wait for 5 milliseconds\n");}
        //         }
        // }
        {
        queue.dequeue(moment.frame);
        }

        if (guidance_phase == true){                                                        // 处置先导段切出状态机定时
            auto now_1 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_1 = now_1 - start_1;
            if(elapsed_1.count() > guidance_time){guidance_phase = false;}
        }

        // printf("2guidance_time :%d, enter_endingpoint :%d\n", guidance_time, enter_endingpoint);     // 强制进入endingpoint测试用
        // auto now_2 = std::chrono::high_resolution_clock::now();                                      // 测试用
        // std::chrono::duration<double> elapsed_2 = now_2 - start_1;                                   // 测试用
        // if (guidance_phase == false && (elapsed_2.count() < (force_entern_endpoint + guidance_time)))// 测试用
        // if (guidance_phase == false && enter_endingpoint == false)                                   // 测试用

        /** 避障规划阶段*/
        // if (guidance_phase == false && ((endpoint_distance/(limit.constraint_speed)) > end_scope))                // 正常完成先导段定时时进入 否则进入else 时间切换方式
        if (guidance_phase == false && endpoint_distance > (end_switch_dis*3)/*十米一算*/&& non_return_state_machine == false)                // 正常完成先导段定时时进入 否则进入else
        {   
            auto start = std::chrono::high_resolution_clock::now();                                             // 记录开始时间用于测算单次路径规划的耗时
            // unsigned int frame = moment.frame;                                                                  // 获取当前帧
            // if(!queue.dequeue(matrix))printf("current sequence pop go wrong\n");                             // TODO这里需不需要判断一下cycbuffer中还有多少余量呢
            queue.atomicity = 0;                                                                                // 锁定cycbuffer的原子时间，期间不可dequeue以确保通过frame偏移访问cycbuffer的一致性 算法单次规划基于的数据是一致可微的 手动dequeue就不存在这个问题了
            // printf("in first invoking set !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            set3d target = queue.invoking(0/*frame*/, (ID));                                                    // 获取目标当前位置 0 就表示最新的目标位置
            // printf("target_xyz: %f:%f:%f\n", target.x,target.y,target.z);
            auto vector_seg = segmentVector(position, SET3D_TO_VEC3D(target), limit.constraint_speed, endpoint_distance);          // 向量分段 <vec3d> vector_seg (包含0位置)
            if(endpoint_distance < end_switch_dis){continue;}                                                   // 立即跳出切换至舞步
            // printf("vector_seg_xyz: %f:%f:%f\n", vector_seg[1].x,vector_seg[1].y,vector_seg[1].z);
            vec3d increment = vector_seg[3] - position;                                                         // TODO 这里可以再把局部放远一点了 取单次局部规划长度 <vec3d> increment  (往后看一个点) 
            Mint guide_target = QUANTIZATION_MAPPING_3D(increment);                                             // 输入一个 vec3d的数据 量化映射到 <Mint> x y z
            AStar::Vec3i guide_target_final = Mint3DToVec3I(guide_target);                                      // 格式转换
            
            // printf("guide_target_final : %d, %d, %d\n", guide_target_final.x,guide_target_final.y,guide_target_final.z);
            // step1 起点终点小数已知 √
            // step2 起点终点映射整数(要考虑舞步也要映射成整数 舞步尺度与我的尺度要相同 舞步的全局坐标要转到我0象归一化坐标 已完成) √
            // step3 生成轨迹解决负方向规划 √
            // step4 整数还原成小数 √
            // step5 加初向量还原原始轨迹 √

            AStar::Generator generator;                                                                             // 定义了一个generator类
            // generator.setWorldSize({100, 100, 100});                                                                // 设置边界范围10m 相对于0.1的量化来说
            generator.setWorldSize({250, 250, 250});                                                                // 设置边界范围25m 相对于0.1的量化来说
            generator.setHeuristic(AStar::Heuristic::euclidean);                                                    // 设置启发函数为欧几里得
            generator.setDiagonalMovement(true);                                                                    // 设置对角元素 这里可以再降点开销

            if (0){}    // 此处预留 //(queue.buffer_count_() < 60){std::this_thread::sleep_for(std::chrono::milliseconds(10));}            // 检查cycbuffer睡眠等待填满
            else{ 
                std::unordered_set<AStar::Vec3i> seenBoxes;                                                         // 定义无序哈希集用于查重 时间复杂度O(1) 最坏退化成O(n)只有在生成同哈希值索引时才会产生内层遍历
                std::unordered_set<AStar::Vec3i> connected_domain;                                                  // 同理定义避碰连通域
                std::vector<AStar::Vec3i> Debugging_observ_howmany_hole;                                            // TODO 调试阶段用
                for (size_t i = 0; i < danceFrame_rate + margin; i++)                                               // TODO 应该设置成动态 向后找多少帧 60帧 这里根据速度约束在单次计算的平均时间开销来推断,尽量的小,避免时序上过长 wall堵塞造成无解的情况
                {   
                    for (size_t j = 0; j < (ALL_DRONE_NUM - 1); j++)                                                // 时间复杂度O(n^2) 检查看看是不是所有飞机都遍历到了
                    {   
                        // printf("inner ALL_DRONE_NUM - 1 !!!!!!!!!!!!!!!:%d\n", j);
                        // printf("danceFrame_rate + margin %d\n", i);
                        // vec3d dyschronism = SET3D_TO_VEC3D(matrix[frame-1+i][j]);                                // 时间上找到障碍帧
                        // printf("in seconed invoking set !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                        vec3d dyschronism = SET3D_TO_VEC3D(queue.invoking(i, j));
                        auto range = manhattanDistance(position, dyschronism).first;    // 返回距离差 x+y+z
                        auto spot = manhattanDistance(position, dyschronism).second;    // 返回距离差向量 差diff vec3d xyz
                        if (range < R_manhattanball/*这里选取障碍范围*/)                                                // 找当前位置相邻范围 TODO 06.21待讨论 思考：用曼哈顿距离 后 会不会引入更多的 非同层点的投影 以此影响有解的可能性 曼哈顿距离和欧拉距离的适用场景 欧拉距离改成曼哈顿距离 帧筛选 这里设置规避的障碍半径 这里的 8 应该用速度约束来控
                        //                                                                                                 这里可以优化的是 不用把方向向量的负球面的那些向量也纳入进来占用遍历时间(需要 因为无解倒退的情况也需要规避)
                        //                                                                                                 目标位置不能和障碍是同一个 要互斥
                        {
                            auto Box = Mint3DToVec3I/*格式转换*/(QUANTIZATION_MAPPING_3D(spot));
                            // printf("Box : %d, %d, %d\n", Box.x,Box.y,Box.z);
                            // if (Box == guide_target_final || Box == zero) continue;                              // 屏蔽此刻 始末 位置有飞机占位                         // 每个位置上做避碰半径拓展，看看能不能把流程放在if后，类型转换前
                            if (seenBoxes.insert(Box).second) {
                            for (size_t i = 0; i < hole.size(); i++)
                                {   
                                Debugging_observ_howmany_hole.push_back(Box);       // TODO 调试阶段用 存储所有飞机中心坐标
                                connected_domain.insert(Box + hole[i]);                        // TODO 06.28 考虑包围进0,0点堵死的情况 减少冗余计算和重复部分的生成是目的 在欧氏距离小于最小穿越距离1.4m以内的质点作为连通域 使用DFS或BFS搜索 不见得三维空间中做搜索的时间开销会低于现在 
                                // printf("Box : %d, %d, %d\n", (Box + hole[i]).x,(Box + hole[i]).y,(Box + hole[i]).z);
                                }                                                       // TODO 07.22 同上 考虑离飞机很近的hole面积，排除掉
                            }
                        }
                    }
                }
                for (AStar::Vec3i each : connected_domain)  {generator.addCollision(each);}                                 // 将去重的扩增点再添加进 Collision 减少寻路的内层while重复遍历
                                                          
                densimeter = Debugging_observ_howmany_hole.size();                                                          // 返回本轮障碍飞机点集数量
                // printf("densimeter: %d\n",densimeter);
                queue.atomicity = 1;                                                                                        // 释放cycbuffer的原子时间
                for (size_t i = 0; i < hole.size(); i++){generator.removeCollision(zero + hole[i]);}                        // 去掉起点本体占位的hole           
                for (size_t i = 0; i < hole.size(); i++){generator.removeCollision(guide_target_final + hole[i]);}          // 先不去掉终点 终点如果有飞机会回退

                /** 开始计算局部路径*/
                // std::cout << "Generate path ... \n";
                auto path = generator.findPath({0, 0, 0}, (guide_target_final));                                            // 库输出路径
                std::reverse(path.begin(), path.end());                                                                     // 反向vector还可以继续优化计算开销
                if (path.back() == guide_target_final){inversePlanning = false;bad_quadrantDrone_num = -1;}                 // 检查本次是否解正常 这里暂时没有考虑去掉终点障碍围堵的情况
                else{   
                        failPlanning_count++;
                        inversePlanning = true;
                        bad_quadrantDrone_num++;
                    }

                std::vector<vec3d> output;                                           
                auto max_point = path.size();
                    {   
                    for (size_t i = 1; i < max_point; i++)
                    {   vec3d temp;
                        temp.x = position.x + (INVERMAPPING(path[i])).x;                        // 对路径反浮点化
                        temp.y = position.y + (INVERMAPPING(path[i])).y;
                        temp.z = position.z + (INVERMAPPING(path[i])).z;
                        output.push_back(temp);
                    }
                    guider.update(output, moment.frame);

                    auto now = std::chrono::high_resolution_clock::now();                       // 单次规划结束时间
                    std::chrono::duration<double> elapsed = now - start;
                    solution_time = elapsed.count();
                    // printf("New path getting!!!!!!!!!!!!\n");
                    // if (cont_planning > 300) enter_endingpoint = true;     // 测试用

                    /** 开启发送业务线程*/
                    if (is_send_dataInplanning == false)
                        {   
                            is_send_dataInplanning = true;
                            send_dataInplanning = std::thread(std::bind(&AlgorithmMng::send_guidance_data, this, std::ref(guider)));      // sub_开启一个避障段发送guider向量的子线程
                            // send_dataInplanning.detach();
                            if (send_dataInplanning.joinable()) {send_dataInplanning.detach();}
                        }                         
                    }
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));     // 每次休眠10毫秒 可变
        }

        else if (guidance_phase == true &&  non_return_state_machine == false)                /** 初始制导阶段*/
        {   
            next_frame += frame_duration;
            singleSend_msg.pos[0] = static_cast<float> (position.x);
            singleSend_msg.pos[1] = static_cast<float> (position.y);
            singleSend_msg.pos[2] = static_cast<float> (position.z + guidance_ascent_speed);
            RGB_control(singleSend_msg, 1);
            send_planningPosition(&singleSend_msg);
            printf("Guideance up px:%f ,py:%f ,pz:%f\n", singleSend_msg.pos[0], singleSend_msg.pos[1], singleSend_msg.pos[2]);
            std::this_thread::sleep_until(next_frame);
            // std::this_thread::sleep_for(std::chrono::milliseconds(33));         // 动态休眠以降低CPU开销
            //                                                                        TODO 增加误解情况用上一次的值或原地等待    
        }
        else                                            /** 末端制导阶段*/
        {   
            if (is_send_dataInplanning == true && depletion == true){        // kill 上一个状态机发送业务线程
                is_send_dataInplanning = false;
                non_return_state_machine = true;
                // std::unique_lock<std::mutex> lk(is_send_dataInplanning_cv_mtx);
                // is_send_dataInplanning_cv.wait(lk, [this]{ return !(is_send_dataInplanning)/*true时继续执行*/; });  //怕出错也可以不要这部分机制
                // pthread_cancel(&send_dataInplanning);
                // pthread_join(&send_dataInplanning, nullptr);
            }
            else{std::this_thread::sleep_for(std::chrono::milliseconds(33));continue;} // 等待depletion置位 把上次规划的路线跑完

            set3d target = queue.invoking( moment.frame, (ID-1));
            singleSend_msg.pos[0] = static_cast<float> (target.x);
            singleSend_msg.pos[1] = static_cast<float> (target.y);
            singleSend_msg.pos[2] = static_cast<float> (target.z);
            // RGB_control(singleSend_msg, 3, target);
            RGB_control(singleSend_msg, 3);
            send_planningPosition(&singleSend_msg);
            printf("Endpoint px:%f ,py:%f ,pz:%f\n", singleSend_msg.pos[0], singleSend_msg.pos[1], singleSend_msg.pos[2]);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));         // 动态休眠以降低CPU开销
        }

        // step3 计算欧氏距离,大致预测到达时间
        // step4 和目标位置的一个向量
        // step6 distanceTotarget
        // 根据获取的当前时间,和丢失ID,推演跟踪时间
        // 生成全局方位向量
     
    }
        guide_finish = true;       // 置计算成功标志位
}

// TODO 异常处理列表 枚举 结构体 函数句柄 根据case调用不同的函数指针
// TODO 对于连续两次回退的处理 升高飞机
// /*输出轨迹是分段的,是否接入正常舞步,计算延迟(用于约束补偿),当前处于上升段还是规划段*/
