#ifndef __ALGORITHMMNG_H_
#define __ALGORITHMMNG_H_


#include <thread>
#include <string>
#include "formation.hpp"

struct PathData {
    std::vector<set3d> road;  // 前继路径
    unsigned int startFrame;            // 起始帧
    unsigned int endFrame;              // 结束帧

    // 初始化构造函数
    PathData(const std::vector<set3d>& r)
        : road(r), 
          startFrame(r.empty() ? 0 : r.front().frame), 
          endFrame(r.empty() ? 0 : r.back().frame) {}
};


class AlgorithmMng {

public:
    AlgorithmMng();
    ~AlgorithmMng();

    void start();
    void stop();

    void init_await_group(void);
    void planning();   // 串行规划
    void guidance(await_drone&);
    void avoidance(await_drone&);
    void interlinkage(await_drone&);
    void timer();       // 超时检测器线程
    std::vector<set3d> check_expath(unsigned int);

    bool spoiler(set3d&, std::vector<set3d>&);
    // set3d transformToWorld();
    // std::tuple<bool, bool, double, set3d> spoiler(set3d&, std::vector<set3d>&); // 搅局者
    void interp(await_drone&, const set3d&, const std::string = "use_for_estimate", int n = 30);
    void package();
    // void planning(CircularQueue& queue/*轨迹表*/, int& ID/*丢失的droneID*/, const vec3d& position/*当前位置*//*, Guide_vector& /*输出位置*/, const pps& moment/*时间戳*/, constraint limit/*飞机各类约束*/);    // 虚拟位置 速度乘以指导向量 = 实际位置 速度用的是打卡速度 

    
public:
    std::string path = "./dac_data";                        // 舞步路径
    std::vector<await_drone> await_group;                   // 待命飞机信息
    FileDescriptorManager* manager;                         // 初始化 manager 指针
    std::vector<PathData> expath;                 // 前继路径

private:
    int finish_count = 0;
    int total_quantity = 30;                                     // 全部待补数量
    int expathStartframe;
    int expathEndframe;

    bool guidance_phase = true;                             // 初始制导阶段
    int guidance_time = 1;                                  // 初始制导上升时间 s
    double guidance_ascent_speed = 1.5;                     // 初始上升速度 m/帧
    double end_switch_dis = 1.4;                            // 切换至末端状态机的剩余距离 舞步约束半径内 一定没有其他飞机了
    double endpoint_distance = 1000;                        // 当前距离终端位置的距离

    int largest_frame = 2800;                               // 本次舞步总帧数
    double framerate = 30;				                    // 帧速率
    double pixels;					                        // guide向量的步长分辨率
    int margin = 5;                                         // 时空上的障碍飞机裕量

    // int calcu_times;				                        // 累计插值次数
    // double elapsed_time;			                        // 累计时间

    bool whether_intrude = false;                           // 路线上是否有闯入者
    static constexpr double constraint_Lspeed = 3.0;		// 合成最大速度约束
    static constexpr double constraint_Hspeed = 6.0;		// 合成最大速度约束
    double collision_radius = 0.4;	            	        // 避碰半径
    double destination_radius = 0.6;	            	        // 避碰半径 为了排除一种量化抖动下打范围扩增


private:
    /** 补位线程 */
    std::thread logThread;
    std::thread dacPackageThread;
    std::thread planningThread;
    std::thread exception_handlingThread;

};

#endif

