#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "formation.hpp"


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

std::vector<drone> drone_group;                                                     // 创建Drone容器
void set_per_drone(size_t control_bit, drone& per, std::vector<std::string> row)     // 放置Drone数据
{   
    static int finish_Xinput_flag = 0, finish_Yinput_flag = 0;
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
            finish_Xinput_flag = 1;
            break;
        }
        case 1:
        if (finish_Xinput_flag == 1)
        {
            for (size_t t = 0; t < row.size(); t++)
            {   
                per.SET[t].y = new_row[t];
            } 
        } 
        finish_Yinput_flag = 1;
        break;
        case 2:
        if (finish_Xinput_flag == 1 && finish_Yinput_flag == 1)
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


int main() {
    auto& view = drone_group;
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
            return 1;
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
    printf("finish input each drone and each frame");
    
    // 设置三个通道 舞步, 经过算法的变轨(不代表实际位置,视为满足位置约束),计算的补位飞机轨迹

}