#include "formation.hpp"
#include "AStar.hpp"
#include "algorithmmng.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// void plot_set3d_points(const std::vector<set3d>& points, <set3d>& goal) {
//     std::vector<double> x_vals, y_vals, z_vals;
//     // std::vector<std::string> colors;

//     for (const auto& point : points) {
//         x_vals.push_back(point.x);
//         y_vals.push_back(point.y);
//         z_vals.push_back(point.z);

//         // 将 RGB 转换为十六进制的颜色表示，如 "#RRGGBB"
//         // char hex_color[8];
//         // snprintf(hex_color, sizeof(hex_color), "#%02x%02x%02x", point.R, point.G, point.B);
//         // colors.push_back(std::string(hex_color));
//     }

//     // 画出散点图
//     // for (size_t i = 0; i < points.size(); ++i) {
//     //     std::vector<double> x(1, x_vals[i]);  // 单个点的 x 坐标
//     //     std::vector<double> y(1, y_vals[i]);  // 单个点的 y 坐标
//     //     std::vector<double> z(1, z_vals[i]);  // 单个点的 y 坐标
//     //     plt::scatter(x, y, z);  // 传递单个点和对应的颜色
//     // }

//     // 设置图形的细节
//     // plt::title("Scatter Plot of set3d Points");
//     // plt::xlabel("X Axis");
//     // plt::ylabel("Y Axis");
//     // plt::set_zlabel("Z Axis");
//     plt::scatter(x_vals, y_vals, z_vals);  // 传递单个点和对应的颜色
//     plt::show();

// }

void plot_set3d_points(const std::vector<set3d>& points, const set3d& goal) { 
    std::vector<double> x_vals, y_vals, z_vals;
    std::vector<double> x_goal, y_goal, z_goal;
    // 处理普通点
    for (const auto& point : points) {
        x_vals.push_back(point.x);
        y_vals.push_back(point.y);
        z_vals.push_back(point.z);
    }

    x_vals.push_back(goal.x);
    y_vals.push_back(goal.y);
    z_vals.push_back(goal.z);
    // 绘制普通点
    plt::scatter(x_vals, y_vals, z_vals);

    // x_goal.push_back(goal.x);
    // y_goal.push_back(goal.y);
    // z_goal.push_back(goal.z);

    // // 处理并绘制目标点，将目标点标记为红色
    // // plt::scatter({goal.x}, {goal.y}, {goal.z}, 50.0, {{"color", "red"}});
    // plt::scatter(x_goal, y_goal, z_goal, 5.0);

    plt::show();
}

#define EXPAND_MAPPING_3Dvec(origin) expandMapping(origin, 0.1)             // 质点体积扩增

#define QUANTIZATION_H_L_3D(origin, hl) QUANTIZATION_H_L_3D_IMPL(origin, hl)
#define QUANTIZATION_H_L_3D_IMPL(origin, hl) QUANTIZATION_##hl(origin)
#define QUANTIZATION_L(origin) quantizationMapping(origin, 0.1)     // 这里用到了宏拼接特性
#define QUANTIZATION_H(origin) quantizationMapping(origin, 0.2)

#define INVERMAPPING_H_L(origin, hl) INVERMAPPING_H_L_3D_IMPL(origin, hl)
#define INVERMAPPING_H_L_3D_IMPL(origin, hl) INVERMAPPING_##hl(origin)
#define INVERMAPPING_L(origin) inverMapping(origin, 0.1)     // 反变换需与变换的映射率一致
#define INVERMAPPING_H(origin) inverMapping(origin, 0.2)     // 使用 0.2 的反变换

#define R_manhattanball_H R_manhattanball_Mapping(AlgorithmMng::constraint_Hspeed)
#define R_manhattanball_L R_manhattanball_Mapping(AlgorithmMng::constraint_Lspeed)
#define R_manhattanball_H_Amend 15.48   // 此处打1.5倍半径系数 1.72 × 1.5 × R=6 舞步飞机3m/s 本体飞机6m/s 共9m/s

// #define shift_middle_road(drone) \
//     drone.middle_road.insert(drone.middle_road.end(), drone.section_road.begin(), drone.section_road.end())

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
// 注意在哈希比较两个 Vec3i 对象的每个坐标是否相等时必须先整型量化 实型只能考虑范数球临域 实型不是离散的 不能比较哈希相等
        }
    };
}

void plot_set3d_points2(std::vector<AStar::Vec3i> points, std::vector<AStar::Vec3i> old_path, const AStar::Vec3i& destination, const AStar::Vec3i& goal) {
    std::vector<int> x_vals, y_vals, z_vals;
    // std::vector<int> x_dest, y_dest, z_dest;
    // std::vector<int> x_goal, y_goal, z_goal;
    // std::vector<std::string> colors;
    // 本轮障碍
    for (const auto& point : points) {
        x_vals.push_back(point.x);
        y_vals.push_back(point.y);
        z_vals.push_back(point.z);
    }
    // 旧路径显示
    for (const auto& element : old_path) {
        x_vals.push_back(element.x);
        y_vals.push_back(element.y);
        z_vals.push_back(element.z);
    }
    // 分段点
    x_vals.push_back(destination.x);
    y_vals.push_back(destination.y);
    z_vals.push_back(destination.z);
    // 终点
    x_vals.push_back(goal.x);
    y_vals.push_back(goal.y);
    z_vals.push_back(goal.z);

    plt::scatter(x_vals, y_vals, z_vals);  // 传递单个点和对应的颜色
    plt::show();

}


void plot_set3d_points3(std::vector<AStar::Vec3i> points,const AStar::Vec3i& goal) {
    std::vector<int> x_vals, y_vals, z_vals;
    std::vector<double> x_goal, y_goal, z_goal;
    // std::vector<std::string> colors;

    for (const auto& point : points) {
        x_vals.push_back(point.x);
        y_vals.push_back(point.y);
        z_vals.push_back(point.z);

        // 将 RGB 转换为十六进制的颜色表示，如 "#RRGGBB"
        // char hex_color[8];
        // snprintf(hex_color, sizeof(hex_color), "#%02x%02x%02x", point.R, point.G, point.B);
        // colors.push_back(std::string(hex_color));
    }
        x_vals.push_back(goal.x);
        y_vals.push_back(goal.y);
        z_vals.push_back(goal.z);

    plt::scatter(x_vals, y_vals, z_vals);  // 传递单个点和对应的颜色
    plt::show();

}


// 重载 + 运算符
set3d operator+(const set3d& lhs, const set3d& rhs) {
    return set3d(
        lhs.x + rhs.x,
        lhs.y + rhs.y,
        lhs.z + rhs.z,
        lhs.frame + rhs.frame,  // 假设帧也需要相加，具体逻辑可以修改
        rhs.R,
        rhs.G,
        rhs.B,
        rhs.W);
}

// 重载 - 运算符
set3d operator-(const set3d& lhs, const set3d& rhs) {
    return set3d(
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.z - rhs.z,
        lhs.frame - rhs.frame,  // 假设帧也需要相减，具体逻辑可以调整
        lhs.R,  // 对于 RGBW 通道，可能需要根据需求决定是否减法
        lhs.G,
        lhs.B,
        lhs.W);
}

// 重载 == 运算符
bool operator==(const set3d& lhs, const set3d& rhs) {
    return (lhs.x == rhs.x &&
            lhs.y == rhs.y &&
            lhs.z == rhs.z &&
            lhs.frame == rhs.frame &&
            lhs.R == rhs.R &&
            lhs.G == rhs.G &&
            lhs.B == rhs.B &&
            lhs.W == rhs.W);
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
}        // 0.4半径共257个点

// std::vector<AStar::Vec3i> expandMapping(const double origin, double rate) {      // 这种保留外壳的方式会留有孔洞
//     int result = static_cast<int>(origin * (1 / rate));  // 计算半径
//     std::vector<AStar::Vec3i> hole;
//     AStar::Vec3i block;

//     for (int i = -result; i <= result; i++) {
//         for (int j = -result; j <= result; j++) {
//             for (int k = -result; k <= result; k++) {
//                 int distSquared = i * i + j * j + k * k;
//                 if (distSquared <= result * result) {
//                     // 检查是否有相邻的体素超出球体半径
//                     if ((i - 1) * (i - 1) + j * j + k * k > result * result ||
//                         (i + 1) * (i + 1) + j * j + k * k > result * result ||
//                         i * i + (j - 1) * (j - 1) + k * k > result * result ||
//                         i * i + (j + 1) * (j + 1) + k * k > result * result ||
//                         i * i + j * j + (k - 1) * (k - 1) > result * result ||
//                         i * i + j * j + (k + 1) * (k + 1) > result * result) {
                        
//                         block.x = i;
//                         block.y = j;
//                         block.z = k;
//                         hole.push_back(block);  // 只保留球体外表面的体素
//                     }
//                 }
//             }
//         }
//     }
//     return hole;
// }       // 0.4半径共134个点


set3d inverMapping(const AStar::Vec3i origin, double rate) {
    set3d result;
    result.x = origin.x * rate;
    result.y = origin.y * rate;
    result.z = origin.z * rate;
    return result;
}

AStar::Vec3i quantizationMapping(const set3d origin, double rate) {
    AStar::Vec3i result;
    result.x = ((int)origin.x) * (1 / rate) + (int)((origin.x - (int)origin.x) * (1 / rate)) % 10;
    result.y = ((int)origin.y) * (1 / rate) + (int)((origin.y - (int)origin.y) * (1 / rate)) % 10;
    result.z = ((int)origin.z) * (1 / rate) + (int)((origin.z - (int)origin.z) * (1 / rate)) % 10;
    return AStar::Vec3i(result.x, result.y, result.z);
}

double euclideanDistance(const set3d& point1, const set3d& point2) {
    double dx = pow(point1.x - point2.x, 2);
    double dy = pow(point1.y - point2.y, 2);
    double dz = pow(point1.z - point2.z, 2);
    return sqrt(dx + dy + dz);
}

std::pair<double, set3d> manhattanDistance(const set3d& point1/*本体飞机*/, const set3d& point2/*障碍飞机*/) {
    double dx = point2.x - point1.x;
    double dy = point2.y - point1.y;
    double dz = point2.z - point1.z;
    return std::make_pair(std::fabs(dx) + std::fabs(dy) + std::fabs(dz), set3d{dx, dy, dz, 0, 0, 0, 0});
}

double R_manhattanball_Mapping(double constraint_speed) {
    return (1.72 * constraint_speed); //R_manhattanball
}

void AlgorithmMng::interp(await_drone& current_drone, const set3d& B, const std::string cmd, int n) {

    // points.reserve(n + 2);                       // 预留内存 A点、B点以及n个插值点，总共n+2个点
    // current_drone.section_road.push_back(A);     // 不需要重复将起点A加入到结果中
    set3d A = current_drone.middle_road.back();
    current_drone.section_road.clear();

    // 计算每个坐标的增量
    double deltaX = (B.x - A.x) / n ;
    double deltaY = (B.y - A.y) / n ;
    double deltaZ = (B.z - A.z) / n ;
    unsigned int deltaFrame = 1;

    // 插入 n 个等距点
    for (int i = 1; i <= n; ++i) {
        set3d interpolatedPoint(
            A.x + deltaX * i,
            A.y + deltaY * i,
            A.z + deltaZ * i,
            A.frame + deltaFrame,
            A.R, A.G, A.B, A.W  // 颜色保持与A点相同
        );
        // printf("interpolatedPoint:%f,%f,%f,%d\n",interpolatedPoint.x,interpolatedPoint.y,interpolatedPoint.z,interpolatedPoint.frame);
        deltaFrame ++;
        current_drone.section_road.push_back(interpolatedPoint);
    }
    // 更具命令字 如果用于线性插值段 即直接赋值
    if (cmd == "direct")
    {
        current_drone.shift_middle_road();
    }
    
    // auto view = current_drone.section_road.size();
    // printf("section_road.back():%f,%f,%f,%d\n",current_drone.section_road.back().x,current_drone.section_road.back().y,current_drone.section_road.back().z,current_drone.section_road.back().frame);
}

std::pair<double, std::vector<set3d>> segmentVector(const set3d& start, const set3d& end, double speed) {              // 长距离分段 speed 表示 speed
    
    std::vector<set3d> segments;
    double totalDistance = euclideanDistance(start, end);                   // 因为涉及到速度约束 这里似乎只能用欧拉距离了
    int numSegments = static_cast<int>(std::ceil(totalDistance / speed));   // 得到段数
    
    set3d direction = { (end.x - start.x) / totalDistance,                  // direction 方向向量 斜边满足速度约束 正交边一定满足
                        (end.y - start.y) / totalDistance,
                        (end.z - start.z) / totalDistance, 0, 0, 0, 0};
    
    for (int i = 0/*0包含初始点1不包含*/; i <= numSegments/*numSegments*/; ++i) {                                // TODO 分段向量超出速度约束的部分 不用再做计算 目前只取 3 段
        set3d segment = { start.x + direction.x * speed * i,
                          start.y + direction.y * speed * i,
                          start.z + direction.z * speed * i };
        segments.push_back(segment);
    }

    // Ensure the end point is included 只取前三段 所以不用末端处理 // TODO 可能要末端处理
    if (euclideanDistance(segments.back(), end) > 1e-6) { // Avoid floating point comparison issues  
        segments.pop_back();        // 先移除最后一个元素
        segments.push_back(end);    // 替换成真值末点元素
    }
    return std::make_pair(totalDistance, segments);
}

// void AlgorithmMng::transformToWorld(){


// }
// std::pair<double, set3d> AlgorithmMng::spoiler(set3d& drone, std::vector<set3d>& sequence, const std::string cmd) {
//     std::pair<double, set3d> result;
//     whether_intrude = false;

//     for (size_t i = 0; i < sequence.size(); i++) {
//         auto [range, spot] = manhattanDistance(drone, sequence[i]);

//         if (range < R_manhattanball_L) {
//             whether_intrude = true;
//             result = std::make_pair(range, spot);
//         }
//     }

//     if (!cmd.empty()) {return result;}  // 返回 pair: 距离 和 位置
// }

bool AlgorithmMng::spoiler(set3d& drone, std::vector<set3d>& sequence){     // TODO 重写 稠密添加box
    whether_intrude = false;
    for (size_t i = 0; i < sequence.size(); i++)        // 遍历 sequence 考虑冗余
    {
        auto [range, spot] = manhattanDistance(drone, sequence[i]);    // 返回距离差 x+y+z set3d位置
        if (range < R_manhattanball_H_Amend)    
        {
            whether_intrude = true; 
        }
    }
    return whether_intrude;
}

std::vector<AStar::Vec3i> QUANTIZATION_H_L_GROUP (const std::vector<set3d>& old_group){
    std::vector<AStar::Vec3i> new_group;
    AStar::Vec3i element;
    for (const auto& group : old_group) {       
        // 使用 group 的索引来计算 i
        set3d index = group - old_group.back();  // 计算索引 i
        element = QUANTIZATION_H_L_3D(index, L);
        new_group.push_back(element);
    }
    return new_group;
}

// std::tuple<bool, bool, double, set3d> AlgorithmMng::spoiler(set3d& drone, std::vector<set3d>& sequence){     
//     bool whether_intrude = false;
//     bool single_pass = false;
//     double range = 0.0;
//     set3d spot;     

//     for (size_t i = 0; i < sequence.size(); i++) {   // 遍历 sequence 考虑冗余
//         auto [current_range, current_spot] = manhattanDistance(drone, sequence[i]);  // 返回距离差 x+y+z set3d位置
        
//         if (current_range < R_manhattanball_H) {
//             whether_intrude = true;
//             single_pass = true; 
//             range = current_range;  // 更新 range
//             spot = current_spot;    // 更新 spot
//         }
//         else{
//             single_pass = false; 
//             range = current_range;  // 更新 range
//             spot = current_spot;    // 更新 spot
//         }
//     }

//     return std::make_tuple(whether_intrude, single_pass, range, spot);  // 返回是否入侵，距离和位置
// }
void AlgorithmMng::interlinkage(await_drone& current_drone){

    expath.push_back(PathData(current_drone.middle_road));          // 推到AlgorithmMng expath
    unsigned int forward = current_drone.middle_road.front().frame;
    unsigned int backward = current_drone.middle_road.back().frame;

    /** 处理后继*/
    current_drone.section_road.clear();
    for (size_t moment = backward; moment < largest_frame; moment++)             // 这里要
    {
        set3d target = manager->read_eachFrame(moment, current_drone.goal_ID);
        current_drone.section_road.push_back(target);
    }
    current_drone.shift_middle_road();
    // plot_set3d_points(current_drone.middle_road,current_drone.middle_road.back());

    /** 处理前延*/
    current_drone.section_road.clear();
    set3d standby = current_drone.middle_road.front();                       // lengthen
    for (size_t moment = 0; moment < forward; moment++)                     // 这里只做原地延续
    {   

        standby.frame = moment;
        current_drone.total_road.push_back(standby);
        // set3d temp;
        // temp.x = current_drone.middle_road.back().x + (INVERMAPPING_L(path[i])).x;                   // 对路径反浮点化
        // temp.y = current_drone.middle_road.back().y + (INVERMAPPING_L(path[i])).y;
        // temp.z = current_drone.
        // temp.frame = moment + i;                                                                     // 加上时间帧
        // current_drone.section_road.push_back(temp);
        // current_drone.section_road.push_back(lengthen);
    }
    current_drone.shift_total_road();
    // plot_set3d_points(current_drone.total_road,current_drone.total_road.back());

}
void AlgorithmMng::guidance(await_drone& current_drone){
    double random = static_cast<double>(std::rand()) / RAND_MAX * (0.4 - 0.2);
    set3d increment(0.0, 0.0, random/*0.2*/, 1, 255, 255, 255, 255);      // 定义增量
    set3d temp_posi;

    for (size_t i = 0; i < guidance_time; i++)
    {       
        if (current_drone.section_road.empty())     // vector<set3d>
        {temp_posi = current_drone.depart_position;}
        else {temp_posi = current_drone.section_road.back();}

        current_drone.section_road.clear();

        for (size_t j = 0; j < framerate; j++)
        {   
            temp_posi = temp_posi + increment;
            current_drone.section_road.push_back(temp_posi);
        }
        // current_drone.middle_road.insert(current_drone.middle_road.end(), current_drone.section_road.begin(), current_drone.section_road.end());
        // shift_middle_road(current_drone);
        current_drone.shift_middle_road();
    }

    /** 观察*/
    // auto number = current_drone.middle_road.size();
    // for (size_t i = 0; i < number; i++)
    // {printf("current_data: %f \n",current_drone.middle_road[i].z);}
    // printf("lenth of current_drone.middle_road: %d \n",current_drone.middle_road.size());
}

std::vector<set3d> AlgorithmMng::check_expath(unsigned int moment){

    unsigned int ture_moment;
    std::vector<set3d> collision_scope;

    if (!expath.empty()){
        for (PathData eachpath : expath){
            if (moment >= eachpath.startFrame && moment <= eachpath.endFrame){   
                unsigned int ture_moment = moment - eachpath.startFrame;
                collision_scope.push_back(eachpath.road[ture_moment]);
            }
        }
        return collision_scope;
    }
    else return std::vector<set3d>();
}



void AlgorithmMng::planning() {

    // double R_manhattanball = 1.72 * constraint_Hspeed;
    AStar::Vec3i zero = {0, 0, 0};                                                          // 定义质点体积补偿hole 复用: 1 起始点 2 质点扩增初始化 // (二维)AStar::Vec2i zero = {0, 0};
    std::vector<AStar::Vec3i> hole = EXPAND_MAPPING_3Dvec(collision_radius);                // 设置安全区hole
    std::vector<AStar::Vec3i> hole_Largescope = EXPAND_MAPPING_3Dvec(destination_radius);                // 设置安全区hole
    std::vector<set3d> plot_all;

    while (!await_group.empty())
    {   
        endpoint_distance = 10000;                        // 重置到终点距离
        auto start = std::chrono::high_resolution_clock::now();static int timeRounds = 1;// 轮次计时
        await_drone current_drone = await_group.front();  // 获取第一个 drone 对象

        {std::cout << "Drone " << timeRounds << " information:" << std::endl;
        std::cout << "  depart_time.frame: " << current_drone.depart_time.frame << std::endl;
        std::cout << "  depart_position: (" 
                  << current_drone.depart_position.x << ", " 
                  << current_drone.depart_position.y << ", " 
                  << current_drone.depart_position.z << ", " 
                  << current_drone.depart_position.frame << ")" << std::endl;
        std::cout << "  goal_ID: " << current_drone.goal_ID << std::endl;
        std::cout << "  self_ID: " << current_drone.self_ID << std::endl;
        std::cout << std::endl;}  // 空行用于区分每个 current_drone 的输出
        guidance(current_drone);

        while (endpoint_distance > end_switch_dis/*2*/) {              // 条件中endpoint_distance是上次的 正常完成先导段定时时进入 否则进入else

            unsigned int moment = current_drone.middle_road.back().frame;
            std::vector<set3d> sequence_target = manager->read_eachFrame(moment + framerate);   // 预测序列 帧号随机访问 
            std::vector<set3d> sequence_now = manager->read_eachFrame(moment);          // 当前序列

            std::vector<set3d> A = check_expath(moment);
            sequence_now.insert(sequence_now.end(), A.begin(), A.end());

            set3d target = sequence_target[current_drone.goal_ID];                      // 取目标predict位置
            set3d real = sequence_now[current_drone.goal_ID];                           // 取目标real位置

            if (spoiler(current_drone.middle_road.back(), sequence_now))                // 6米内是否有飞机
            {
                auto [remain_dis, vector_seg] = segmentVector(current_drone.middle_road.back(), target, constraint_Hspeed);/*endpoint_distance = remain_dis;*/
                set3d increment;
                if (vector_seg.size() > 1){
                    interp(current_drone, vector_seg[1]);                               // 改进 先算出直飞线性插值点
                    increment = vector_seg[1] - current_drone.middle_road.back();       // 取分段增量
                }else {
                    interp(current_drone, vector_seg[0]);                               // 改进 先算出直飞线性插值点
                    increment = vector_seg[0] - current_drone.middle_road.back();       // 取分段增量
                }
                std::vector<AStar::Vec3i> old_path = QUANTIZATION_H_L_GROUP(current_drone.middle_road);
                AStar::Vec3i destination = QUANTIZATION_H_L_3D(increment, L);           // 这一步是决定速度的 因为一帧只走一个体素
                AStar::Vec3i goal = QUANTIZATION_H_L_3D((target - current_drone.middle_road.back()),L);
                AStar::Generator generator;                                             // 注意生命周期
                // generator.setWorldSize({1, 1, 1});
                generator.setWorldSize({100, 100, 100});                                // 设置边界范围10m 相对于0.1的量化来说 
                generator.setHeuristic(AStar::Heuristic::euclidean);                    // 设置启发函数为欧几里得
                generator.setDiagonalMovement(true);  
                std::unordered_set<AStar::Vec3i> seenBoxes;                             // 定义无序哈希集用于查重 时间复杂度O(1) 最坏退化成O(n)只有在生成同哈希值索引时才会产生内层遍历
                std::unordered_set<AStar::Vec3i> connected_domain;                      // 同理定义避碰连通域
                std::vector<AStar::Vec3i> see;

                for (size_t i = 0; i < framerate; i++)                                               // TODO 1 先不考虑后退打情况 2应该设置成动态 向后找多少帧 60帧 这里根据速度约束在单次计算的平均时间开销来推断,尽量的小,避免时序上过长 wall堵塞造成无解的情况
                {  
                    std::vector<set3d> sequen = manager->read_eachFrame(moment + i); 

                    std::vector<set3d> A = check_expath(moment + i);
                    sequen.insert(sequen.end(), A.begin(), A.end());

                    // if(sequen.size() != 10)
                    // {printf("current_data over\n");}
                    for (size_t j = 0; j < sequen.size(); j++)
                    {   

                        auto [range, spot] = manhattanDistance(current_drone.section_road[i], sequen[j]);
                        if (range < R_manhattanball_L)
                        {   
                            set3d true_spot = (current_drone.section_road[i]-current_drone.middle_road.back()) + spot;    // 转换到规划起点的系                           
                            AStar::Vec3i Box = QUANTIZATION_H_L_3D(true_spot, L);
                            if (seenBoxes.insert(Box).second) {
                            for (size_t i = 0; i < hole.size(); i++)
                                {   
                                // auto viewx = seenBoxes.size();
                                connected_domain.insert(Box + hole[i]);                        // TODO 06.28 考虑包围进0,0点堵死的情况 减少冗余计算和重复部分的生成是目的 在欧氏距离小于最小穿越距离1.4m以内的质点作为连通域 使用DFS或BFS搜索 不见得三维空间中做搜索的时间开销会低于现在 
                                see.push_back(Box + hole[i]);
                                }                                                       // TODO 07.22 同上 考虑离飞机很近的hole面积，排除掉
                            }
                        }
                    }
                }         
  
                for (AStar::Vec3i each : connected_domain) {generator.addCollision(each);} /* printf("element:%d,%d,%d,%d\n ",++count, each.x,each.y,each.z);*/
                for (size_t i = 0; i < hole.size(); i++) {generator.removeCollision(zero + hole[i]);}                        // 去掉起点本体占位的hole           
                for (size_t i = 0; i < hole_Largescope.size(); i++) {generator.removeCollision(destination + hole_Largescope[i]);} 

                /** 开始计算局部路径*/
                // std::cout << "Generate path ... \n";
                auto path = generator.findPath({0, 0, 0}, (destination));                                            // 库输出路径
                std::reverse(path.begin(), path.end());
                // auto howmuch = path.size();                                 // TODO 长度大于30点时 ，只取30点 小于30点时取全部

                // plot_set3d_points2(generator.walls, old_path, destination, goal);
                current_drone.section_road.clear(); 
                if(path.size() > framerate){
                    for (size_t i = 1; i <= framerate; i++)        // 这里的1应该是
                        {   set3d temp;
                            temp.x = current_drone.middle_road.back().x + (INVERMAPPING_L(path[i])).x;                   // 对路径反浮点化
                            temp.y = current_drone.middle_road.back().y + (INVERMAPPING_L(path[i])).y;
                            temp.z = current_drone.middle_road.back().z + (INVERMAPPING_L(path[i])).z;
                            temp.frame = moment + i;                                                                     // 加上时间帧
                            current_drone.section_road.push_back(temp);
                        }
                }
                else{
                    for (size_t i = 1; i < path.size(); i++)        // 这里的1应该是
                        {   set3d temp;
                            temp.x = current_drone.middle_road.back().x + (INVERMAPPING_L(path[i])).x;                        // 对路径反浮点化
                            temp.y = current_drone.middle_road.back().y + (INVERMAPPING_L(path[i])).y;
                            temp.z = current_drone.middle_road.back().z + (INVERMAPPING_L(path[i])).z;
                            temp.frame = moment + i; 
                            current_drone.section_road.push_back(temp);
                        }
                }

                current_drone.shift_middle_road();
                double predicted_endpoint = euclideanDistance(current_drone.middle_road.back(), target); 
                endpoint_distance = euclideanDistance(current_drone.middle_road.back(), target);
                // endpoint_distance = euclideanDistance(current_drone.middle_road.back(), real); 
                // plot_set3d_points(current_drone.middle_road,vector_seg[1]);
            }
            else{
                auto [remain_dis, vector_seg] = segmentVector(current_drone.middle_road.back(), target, constraint_Hspeed);endpoint_distance = remain_dis;// (包含0位置) // 向量分段 <vec3d> vector_seg (包含0位置)
                interp(current_drone, vector_seg[1], "direct");
                endpoint_distance = euclideanDistance(current_drone.middle_road.back(), target);        // 向外插值完成后再次检查终点距
                
                
            }
        }
        interlinkage(current_drone);
        plot_all.insert(plot_all.end(), current_drone.total_road.begin(), current_drone.total_road.end());

        // printf("current_data over");
        await_group.erase(await_group.begin());  // 移除第一个 drone 对象
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Elapsed time for round " << timeRounds << ": " << elapsed.count() << " seconds" << std::endl;++timeRounds;
        finish_count ++;
    }
    std::cout << "All drones processed!" << std::endl;
    printf("All finish count: %d\n", finish_count);
    plot_set3d_points(plot_all,plot_all.back());
}


set3d linearInterpolation(const set3d& A, const set3d& B, double t) {
    set3d result;
    result.x = A.x + t * (B.x - A.x);
    result.y = A.y + t * (B.y - A.y);
    result.z = A.z + t * (B.z - A.z);
    return result;
}

std::vector<set3d> insertPointsBetween(const set3d& A, const set3d& B, int point_num) {
    std::vector<set3d> points;

    // 插入 31 个点，t 从 0 变化到 1，分为 30 份
    for (int i = 0; i <= point_num; ++i) {
        double t = static_cast<double>(i) / point_num;  // 计算 t 值
        points.push_back(linearInterpolation(A, B, t));
    }

    return points;
}

// auto size_of_middle = current_drone.middle_road.size();
// auto middle_back = current_drone.middle_road.back();
// printf("view size_of_middle:%d , middle_back:%f,%f,%f,%d, target:%f,%f,%f\n",size_of_middle, middle_back.x,middle_back.y,middle_back.z,middle_back.frame, target.x, target.y, target.z);
// avoidance(current_drone);


/** 把middle量化到int做观察*/ 
// std::vector<AStar::Vec3i> basket;
// for (const auto& element : current_drone.middle_road){
//     AStar::Vec3i Box = QUANTIZATION_H_L_3D(element - current_drone.middle_road.back(), L);
//     basket.push_back(Box);
// }
// for (const auto& element2 : generator.walls)
// {
//     AStar::Vec3i A = element2;
//     basket.push_back(A);
// }
// plot_set3d_points3(basket,goal); 

// if (/*vector 为空*/)
// {
//     interp();
// }
// else
// {
//     AStar
// }

// 添加新的这段轨迹到manager指针对象

// void AlgorithmMng::avoidance(await_drone& current_drone) {
//     unsigned int moment = current_drone.middle_road.back().frame;
//     std::vector<set3d> sequence = manager->read_eachFrame(moment);               // 帧号随机访问
//     set3d target = sequence[current_drone.goal_ID];
//     auto vector_seg = segmentVector(current_drone.middle_road.back(), target, constraint_Hspeed);          // 向量分段 <vec3d> vector_seg (包含0位置)
//     set3d increment = vector_seg[1] - current_drone.middle_road.back();


//     QUANTIZATION_H_L_3D(origin, H); // 调用 quantizationMapping(origin, 0.1)
//     QUANTIZATION_H_L_3D(origin, L); // 调用 quantizationMapping(origin, 0.2)
//     AStar::Vec3i guide_target_final1 = QUANTIZATION_H_L_3D(increment, H);    // increment 为规划空间打终点
//     AStar::Vec3i guide_target_final2 = QUANTIZATION_H_L_3D(increment, L);

//     AStar::Vec3i guide_target_final
// }


// printf("set 3d target: x:%f y:%f z:%f f:%d \n", target[1].x, target[1].y, target[1].z, target[1].frame);
// printf("current_data over");
// /** 先导段 */
// guidance(current_drone);

// /** 避障段 */
// if (range < R_manhattanball_H){whether_intrude = true;}else {continue;}

// for (const auto& element : connected_domain)
// {   static int innernum = 0;
//     set3d temp;
//         temp.x = current_drone.middle_road.back().x + (INVERMAPPING_L(element)).x;                   // 对路径反浮点化
//         temp.y = current_drone.middle_road.back().y + (INVERMAPPING_L(element)).y;
//         temp.z = current_drone.middle_road.back().z + (INVERMAPPING_L(element)).z;
//         temp.frame = 0;                                                                     // 加上时间帧
//         current_drone.section_road.push_back(temp);
//     innernum++;
//     if (innernum > 10){break;}
// }

// printf("current_data over\n");