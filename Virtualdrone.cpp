#include <thread>
#include <chrono>
#include <cmath>
#include <chrono>
#include <iostream>
#include <mutex>
#include "formation.hpp"
#include "AStar.hpp"

void timegoes(pps& moment) {
    while (1)
    {   extern vec3d output;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        moment.frame ++;
        printf("moment now :%d\n", moment.frame);
        // printf("guide_vec now :x %f\n, y %f\n, z %f\n", output.x, output.x, output.z);
    }
}
// 由于惯性 该惯性的迟滞大小和两次方向切线改变角度平方成正比 由此还可以计算能量损耗 
// 速度是有方向的啊                                                                 
void Virtual_location(const std::vector<vec3d>& guide/*当前指导向量*/, vec3d& virtual_posi/*当前虚拟位置*/, const pps& moment, const constraint limit) {       // 改变虚拟位置的时候注意要加锁
        // extern std::mutex mtx_position;         // 声明锁
        // extern std::condition_variable cv;      // 声明线程信号在主函数定义
        // std::unique_lock<std::mutex> lock(mtx_position);    // 锁 virtual_posi 状态
        // cv.wait(lock, []{ return ready; });
        extern bool guide_finish;
        static pps save_moment = {0};   // 初始化静态变量
        static int receive_same_momenttimes = 0; // 初始化计数器

        if (save_moment.frame != moment.frame && guide_finish == true) {
            receive_same_momenttimes = 0;
            save_moment.frame = moment.frame;
        
        auto temp = guide[1] - guide[0];
        std::this_thread::sleep_for(std::chrono::seconds(10));
        printf("stop");
        // auto /*米每帧*/maxDof = limit./*m/s*/constraint_speed / limit./*frame/s*/framerate;             // drone每帧飞行速度 即获取每帧最大飞行距离 这个距离是合成速度约束
        // auto /*米每帧*/maxDof = limit.constraint_speed / limit.framerate;
        // // 这里要加线程锁
        // // 计算路程
        // double temp = 0;
        // double curve_length = 0;
        // for (size_t i = 1; i < guide.size(); i++)
        // {
        //     temp = guide[i] - guide[i-1];
        //     curve_length = curve_length + temp;
        //     if(limit.constraint_speed < curve_length) 
        //     {curve_length = curve_length - temp;
        //         break;
        //     }        // 此时的curve_length就是本秒内最大能达到的地方
        // }
        //  // 最大到达处
        // virtual_posi = guide[i-1];
        }
        else {
            receive_same_momenttimes++;
        }
        // mtx_position.unlock();// 解锁
}     // 虚拟位置 速度乘以指导向量 = 实际位置 速度用的是打卡速度 