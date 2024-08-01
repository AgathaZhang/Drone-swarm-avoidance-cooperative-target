#include <thread>
#include <chrono>
#include <cmath>
#include <chrono>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include "formation.hpp"
#include "AStar.hpp"
#include "planning.hpp"
#include "algorithmmng.h"

void timegoes(pps& moment) {
    while (1)
    {   extern vec3d output;
        std::this_thread::sleep_for(std::chrono::milliseconds(33));         // 模拟每帧的实际时间 33ms
        moment.frame ++;
        // printf("moment now :%d\n", moment.frame);
        // printf("guide_vec now :x %f\n, y %f\n, z %f\n", output.x, output.x, output.z);
    }
}

void receive(AlgorithmMng& am) {
    while (true)
    {   
        mavlink_message_t msg;
        // mavlink_message_t rc;
        // am.onMavlinkMessage(&rc);
        am.handleMsgFromDrone(&msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(300)); 
    }
    

}

// void changed(const vec3d& virtual_posi){
//     extern bool yes_change;
//     auto last_param_value = virtual_posi;
//     while (true) {
//         if (virtual_posi != last_param_value) {
//             yes_change = true;
//             std::cout << "continue planning.\n";
//             last_param_value = virtual_posi;
//         } else {
//             yes_change = false;
//         }
//     }
// }


// void monitor(const vec3d& virtual_posi) {
//     extern bool parameter_changed;
//     extern std::mutex changed;
//     extern std::condition_variable cv;       
//     auto last_param_value = virtual_posi;
//     while (true) {
//         std::unique_lock<std::mutex> lock(changed);
//         if (virtual_posi != last_param_value) {
//             parameter_changed = true;
//             std::cout << "continue planning.\n";
//             last_param_value = virtual_posi;
//             cv.notify_one(); // 通知被挂起的线程
//         } else {
//             parameter_changed = false;
//         }
//     }
// }

// 由于惯性 该惯性的迟滞大小和两次方向切线改变角度平方成正比 由此还可以计算能量损耗 
// 速度是有方向的啊                                                                 
void Virtual_location(const Guide_vector& origin_guide/*当前指导向量*/, vec3d& virtual_posi/*当前虚拟位置*/, const pps& moment, const constraint limit) {       // 改变虚拟位置的时候注意要加锁
        static auto /*米每帧*/maxDof = limit./*m/s*/constraint_speed / limit./*frame/s*/framerate;              // drone每帧飞行速度 即获取每帧最大飞行距离 这个距离是合成速度约束
        extern std::mutex mtx_position;                                                                         // 位置数据锁
        // extern std::mutex mtx_guide_soket;                                                                      // 预测向量锁(弃用)
        extern bool position_update;
        // extern std::vector<vec3d> guide_soket;// 预测向量锁(弃用)
        
        // extern bool guide_finish;
        // extern std::condition_variable cv;                                                                   // 线程信号
        // cv.wait(lock, []{ return ready; });

        // static pps save_moment = {0};                                                                        // 初始化静态变量 (弃用)
        // static int receive_same_momenttimes = 0;                                                             // 初始化计数器 (弃用)
        // if (save_moment.frame != moment.frame && guide_finish == true) {
        //     receive_same_momenttimes = 0;
        //     save_moment.frame = moment.frame;
        auto start = std::chrono::high_resolution_clock::now();
    while (true) // 一直更新
    {   static int printcount = 0;
        printcount++;
        bool changed = true;
        // std::this_thread::sleep_for(std::chrono::milliseconds(33));                                             // 按帧时长更新Virtualdrone
        vec3d old_virtual_posi;                                                                                 // 当前驻点
        std::vector<vec3d> guide;                                                                               // 屏蔽外部guide向量在改变
        pps old_moment;                                                                                         // old_moment表示该次轨迹规划基于哪个时间点做的
        {
            guide = origin_guide.read().first;          // 获取guide向量
            old_moment = origin_guide.read().second;    // 获取基于时间
            old_virtual_posi = virtual_posi;            // 获取当前驻点
            // mtx_guide_soket.lock();// 预测向量锁(弃用)
            // guide_soket = guide;
            // mtx_guide_soket.unlock();
        }

        // 计算路程
        double curve_length = 0;    // 曲线总长
        std::vector<double> pump;   // 打卡点序列
        bool pump_times = true;     // 用于控制pump只累计一次

        while (changed && old_moment.frame != 0)             // TODO flag 获得了  这个while是设置用来等待时间满足 位置更新的
        {   
            if (pump_times == true) {
                for (size_t i = 1; i < guide.size(); i++)                                                   // 进行guide为空检查或者长度检查:if(old_moment.frame == 0) break;这种情况不用考虑 因为guide.size()没有值的时候不进循环 // TODO 0.1 * (15-0) < 0 // 读 当前帧 给出位置  这是关键步骤
                {   
                    // static std::vector<double> pump;   // 打卡点序列
                    auto temp = guide[i] - guide[i-1];
                    curve_length = curve_length + temp.Euler_dis();                                         // 这里会涉及一个追及问题
                    pump.push_back(curve_length); 
                }
                pump_times = false;                                                                         // 让pump.push_back(curve_length)只做一次 防止一直累加
            }
            if(pump.empty() != true)                // TODO这里应该放入vector<guide>长度校验 因为如果只收到本身 或者生成路径失败的话就更新不出位置了，因为最新更新距是0.1
            {   
                double satisfy_dis_now = maxDof * (moment.frame - old_moment.frame);                    // 位置更新的最小移动单位就是帧距离maxDof
                for (size_t i = 0; i < pump.size(); ++i)
                {
                    // curve_length = curve_length - temp.Euler_dis();                      // 最大到达处的距离// 此时的curve_length就是本秒内最大能达到的地方
                    if (satisfy_dis_now >= pump[i]) {
                                                            // 这里应该设置大于最大的比较值
                        mtx_position.lock();
                        virtual_posi = guide[i+1];          // 因为这里的pump[0] 是guide[i] - guide[i-1] 所以 virtual_posi = guide[i+1]; 
                        mtx_position.unlock();
                        changed = false;            // 检测到位置更新
                    }
                    if (satisfy_dis_now < pump[i]) break;
                    // if (i = 0) continue;
                    // else virtual_posi = guide[i-1];}
                    // else{printf("Have a erro\n");}
                }
            }
            else break;     // 防止空路径 不更新跳不出循环
            if (virtual_posi != old_virtual_posi)break;                 // TODO 超时也跳出
        }
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - start;

            if (elapsed.count() >= 2.0) {
            std::cout << "New position time: " << moment.frame << " pos: "<< virtual_posi.x << " " << virtual_posi.y << " " << virtual_posi.z << "\n";
            start = now; // 重置计时器
}
            // cv.notify_one();
            // monitor(virtual_posi);// 已经在新开线程中检测 这里不需要显示调用 virtual_posi 不改变 那么把planning线程挂起
            // changed(virtual_posi);
            

    }
        // virtual_posi = guide[i-1];       
        // receive_same_momenttimes++;
        // mtx_output.unlock();// 解锁
}       // 虚拟位置 速度乘以指导向量 = 实际位置 速度用的是打卡速度 

