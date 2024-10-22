#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "formation.hpp"

int start_frame = 25;				// 开始补位动作帧
double constraint_speed = 6;		// 速度约束
double collision_radius = 1.4;		// 避碰半径
int ALL_DRONE_NUM = 1934;			// 飞机总数

std::string filename_base = "E:\\Project_C\\Yielded_240605\\";              // 舞步路径
std::string map(int num)                                                    // 舞步路径分量
{
    if (num == 0) return("datax.csv");
    if (num == 1) return("datay.csv");
    if (num == 2) return("dataz.csv");
    else throw std::invalid_argument("Invalid control bit value");
} 
std::vector<std::string> split(const std::string& s, char delimiter) {      // 分割字符串入栈
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}
std::vector<double> stringTodouble(std::vector<std::string> row)            // 文本转浮点
{
    std::vector<double> result;
    for (const std::string& str : row) {
        result.push_back(std::stod(str));
    }
    return result;
}

drone::drone(int id) {                                                      // Drone构造函数
		drone_ID = id;
}
// std::vector<std::vector<set3d>> matrix;                                              //放在main文件中定义了 创建时序轨迹表 最外维是所有时间序列 第二维是所有架次 架次中又包含结构体
std::vector<drone> drone_group;                                                      // 创建Drone容器 按per dorne存储
void set_per_drone(size_t control_bit/*xyz分量*/, drone& per/*一架次*/, std::vector<std::string> row/*帧序列*/)     // 放置Drone数据
{   
    // static int finish_Xinput_flag = 0, finish_Yinput_flag = 0;
    auto new_row = stringTodouble(row);
    switch (control_bit)
    {
        case 0:
        {   set3d temp_set3d;
            for (size_t t = 0; t < row.size(); t++)
            {   temp_set3d.x = new_row[t];              // 赋值x分量
                temp_set3d.frame = t+1;                 // 赋值frame分量
                per.SET.push_back(temp_set3d);
            }
            // finish_Xinput_flag = 1;
            break;
        }
        case 1:
        // if (finish_Xinput_flag == 1)
        {
            for (size_t t = 0; t < row.size(); t++)
            {   
                per.SET[t].y = new_row[t];
            } 
        } 
        // finish_Yinput_flag = 1;
        break;
        case 2:
        // if (finish_Xinput_flag == 1 && finish_Yinput_flag == 1)
        {
            for (size_t t = 0; t < row.size(); t++)
            {   
                per.SET[t].z = new_row[t];
            } 
        }
        break;
        default:
            throw std::invalid_argument("Invalid control bit value");
    }
}
void time_series_map(std::vector<std::vector<set3d>>& matrix,const std::vector<drone> drone_group)                        // per drone转 输出舞步时间序列表
{   if (drone_group.empty()) {
        throw std::runtime_error("drone_group is empty");
    }

    size_t num_drones = drone_group.size();
    size_t num_frames = drone_group[0].SET.size();
    matrix.resize(num_frames, std::vector<set3d>(num_drones));

    // 检查drone_group中每个drone的SET大小一致性
    for (const auto& drone : drone_group) {
        if (drone.SET.size() != num_frames) {
            throw std::runtime_error("Inconsistent SET sizes in drone_group");
        }
    }

    for (size_t j = 0; j < drone_group[0].SET.size(); j++) // 循环帧序列
    {   
        for (size_t i = 0; i < drone_group.size(); i++)  // 循环1934架
        {   /*auto view = drone_group[i].SET[j];              // 第i架飞机的第j帧*/
            matrix[j][i] = drone_group[i].SET[j];
        }
    }
    // printf("finished all time series\n");
}
// int Read_frame(std::vector<std::vector<drone>> matrix/*@param 输出的时间序列表*/) {
void Read_frame(std::vector<std::vector<set3d>>& matrix) {
    // auto& view = drone_group;
    for (int i = 0; i < ALL_DRONE_NUM; ++i) {     // 创建1934架次
        drone plane(i+1);
        drone_group.push_back(plane);
    }


    for (size_t sn_choose_xyz = 0; sn_choose_xyz <= 2; sn_choose_xyz++)
    {
        std::string filename = filename_base + map(sn_choose_xyz);
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open the file " << filename << std::endl;
            return;
        }

        std::string line;
        int each = 0;                                           // 帧序列分配给的飞机号
        while (std::getline(file, line)) {                      // 一行代表一架飞机的帧序列
            std::vector<std::string> row = split(line, ',');
            drone& per_drone = drone_group[each];
            // drone* per_drone = &drone_group[each];
            each++;
            set_per_drone(sn_choose_xyz/*控制导入到xyz哪个分量*/, per_drone/*一架次*/, row/*帧序列*/);
        }

        file.close();
    }
    // printf("finish input each drone and each frame as object mode\n");
    time_series_map(matrix, drone_group);
    printf("finish input each drone and each frame as table mode\n");
    // auto view_matrix = matrix[75][500];
    return;
    
    // 设置三个通道 舞步, 经过算法的变轨(不代表实际位置,视为满足位置约束),计算的补位飞机轨迹

}

