#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "formation.hpp"

// Helper function to split a string by a delimiter and return a vector of strings
std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

std::string filename_base = "E:\\Project_C\\Yielded_240605\\";
std::string map(int num)
{
    if (num == 0) return("datax.csv");
    if (num == 1) return("datay.csv");
    if (num == 2) return("dataz.csv");
} 


void set_per_drone(size_t control_bit, drone per)
{
    switch (control_bit)
    {
        case 0:
            // Code for control_bit 0
            break;
        case 1:
            // Code for control_bit 1
            break;
        case 2:
            // Code for control_bit 2
            break;
    }
}

drone::drone(int id) {
		drone_ID = id;
}

int main() {
    std::vector<drone> per_drone;       // 创建机群容器
    for (int i = 0; i < 1934; ++i) {     // 创建1934架次
        drone obj(i);
        per_drone.push_back(obj);
    }


    for (size_t i = 0; i < 3; i++)
    {
        std::string filename = filename_base + map(i);

        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open the file " << filename << std::endl;
            return 1;
        }

        std::vector<std::vector<std::string>> data;     // 二维vector
        std::string line;

        // while (std::getline(file, line)) {
        //     std::vector<std::string> row = split(line, ',');
        //     data.push_back(row);
        // }

        while (std::getline(file, line)) {
            std::vector<std::string> row = split(line, ',');
            for (size_t j = 0; j < row.size(); i++)
            {   
                auto per = per_drone[i];
                // set_per_drone(j, per);
                printf("num:%d", i);
            }
            
        }

        file.close();
    }

    // 设置三个通道 舞步, 经过算法的变轨(不代表实际位置,视为满足位置约束),计算的补位飞机轨迹





    // Print the CSV file content





    // for (const auto& row : data) {
    //     for (const auto& col : row) {
    //         std::cout << col << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // return 0;
}