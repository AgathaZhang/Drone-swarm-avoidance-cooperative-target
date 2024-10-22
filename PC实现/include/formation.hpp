#ifndef __FORMATION__
#define __FORMATION__
// #pragma once
#include <vector>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <string>
#include <unordered_map>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <cstring>
#include <sys/types.h>
#include <arpa/inet.h>
#include <fstream>
#include <sstream>
#include <regex>
#include <thread>
#include <chrono>
#include <unordered_set>
#include <cstdlib>  // For std::rand()
#include <ctime>    // For std::time()
#include <tuple>
#include <../eigen3/Eigen/Dense>
// #include <variant>  // 多返回类型绑定

// extern int start_frame;				// 开始补位动作帧
// extern double constraint_speed;		// 速度约束
// extern double collision_radius;		// 避碰半径

typedef struct pps pps;
struct pps{
	// double time;
	unsigned int frame;
	pps(unsigned int f = 0) : frame(f) {} // 带默认值的构造函数
};

typedef struct set3d set3d;		// 含帧序
struct set3d {
	double	x;
	double	y;
	double	z;
    unsigned int frame;
	uint8_t	R;
	uint8_t G;
	uint8_t B;
	uint8_t W;
	
    // 带参构造函数
    set3d(double x_val = 0, double y_val = 0, double z_val = 0, unsigned int frame_val = 0, uint8_t	R_val = 0, uint8_t G_val = 0, uint8_t B_val = 0, uint8_t W_val = 0)
        : x(x_val), y(y_val), z(z_val), frame(frame_val), R(R_val), G(G_val), B(B_val), W(W_val) {}

};

/** await_drone 类*/
class await_drone {
public:
    pps depart_time;                            // 出发时间
    set3d depart_position;                      // 出发位置
    std::vector<set3d> section_road;            // 区段路径
    std::vector<set3d> middle_road;             // 完整段
    std::vector<set3d> total_road;              // 完整段
    int goal_ID;                                // 目标 ID
    int self_ID;                                // 自我 ID

    // 带参构造函数
    await_drone(unsigned int depart_frame_val = 0, int goal_ID_val = 0, int self_ID_val = 0, set3d depart_position_val = set3d())
        : depart_time(depart_frame_val), // 初始化 depart_time，使用传入的 frame 值
          depart_position(depart_position_val), // 初始化出发位置
          goal_ID(goal_ID_val), // 初始化目标 ID
          self_ID(self_ID_val) // 初始化自我 ID
    {
        // depart_position.frame = depart_time.frame;
    }

    void shift_middle_road() {

        this->middle_road.insert(this->middle_road.end(), this->section_road.begin(), this->section_road.end());
        // for (size_t i = 0; i < section_road.size(); i++)
        // {
        //     printf("after middle_road size: %zu , section_road: %f, %f, %f, %u\n",
        //     this->section_road.size(),
        //     this->section_road[i].x, 
        //     this->section_road[i].y, 
        //     this->section_road[i].z, 
        //     this->section_road[i].frame);
        // }
    }

    void shift_total_road() {

    this->total_road.insert(this->total_road.end(), this->middle_road.begin(), this->middle_road.end());
    
}
};


class FileDescriptorManager {
public:

    FileDescriptorManager(const std::string& directory);
    // ~FileDescriptorManager(); // TODO 释放FD

    size_t initialize(const std::string& directory);                   // 初始化FD
    std::vector<set3d> read_eachFrame(const pps& frame);               // 帧号随机访问
    std::vector<set3d> read_eachFrame(unsigned int frame_num);         // 声明重载接收 unsigned int 类型的参数
    set3d read_eachFrame(const pps& frame, int goal_ID);
    set3d read_eachFrame(unsigned int frame_num, int goal_ID);       // 重载单目标读取
    void read_headFrame(void);                                       // 读头文件帧获取最大帧数信息

private:
    std::vector<int> fileDescriptors_;  // 文件描述符
    std::vector<std::string> files;     // 每个路径名
	size_t capacity;                    // 总数
    set3d get_eachPosition(int index/*架次*/, const pps& frame);        // 访问单机舞步
	// std::mutex readSDcard_mutex;

};

#endif

    // void 可以根据待补个数和距离往后推算时间
    // void 延迟起飞对补位意义打检查 超末端时间未补时 不再补位直接跳过
    // void // 输入当前帧，查出当前位置
    // void export(int frame);			//TODO 给帧数输出对应位置
    // void get_radio(int frame, /*未来几帧轨迹*/);		// TODO 接收广播轨迹




































// void Read_frame(std::vector<std::vector<set3d>>&);			// old declar
// extern FileDescriptorManager manager;
// extern CircularQueue queue;
// void loadInCycque(const pps& first_moment/*初始读写偏移*/, CircularQueue& queue/*循环队列*/);
// void processData(CircularQueue& queue);
// std::vector<set3d> Read_frame(const pps& frame/*帧序*/, FileDescriptorManager& manager/*文件管理器*/);
// void time_series_map(std::vector<std::vector<set3d>>&, const std::vector<drone>);
// extern void socketCommunication(void);
// extern std::vector<std::vector<set3d>> matrix;  
// extern set3d view_matrix;  
// extern bool isSorted;
// void consumeInCycque(const pps& first_moment, CircularQueue& queue); // 剔除旧帧线程函数
// // void consumeInCycque(CircularQueue& queue); //测试消费线程
// #endif


