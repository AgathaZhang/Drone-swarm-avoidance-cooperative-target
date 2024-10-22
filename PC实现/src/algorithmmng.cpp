#include "algorithmmng.h"
#include "formation.hpp"


// #include <errno.h>
// #include <pthread.h>
// #include <signal.h>
// #include <stdbool.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <sys/poll.h>
// #include <time.h>
// #include <unistd.h>
// #include <arpa/inet.h>
// #include <sys/stat.h>

AlgorithmMng::AlgorithmMng()
{   

}

AlgorithmMng::~AlgorithmMng() 
{

}

void AlgorithmMng::start() {

    if (access(this->path.c_str(), F_OK) == -1) {
        printf("Invalid path: %s\n", this->path.c_str());
        return;}    // 检查路径是否有效
    // manager = FileDescriptorManager(this->path);  // 初始化FD管理器
    manager = new FileDescriptorManager(this->path); // 动态分配对象
    init_await_group();
    planningThread = std::thread(std::bind(&AlgorithmMng::planning, this));


}

void AlgorithmMng::stop() {
    /** 补位*/
    planningThread.join();      // 这里join的顺序应该按线程结束释放的先后顺序 先释放的放在前

    if (manager) {delete manager;}    // 这里释放FD 确认所有dac生成完成

}

/** 初始化补位等待飞机 */
void AlgorithmMng::init_await_group(void) {
    // 初始化随机数种子
    std::srand(static_cast<unsigned int>(std::time(0)));  // Seed for randomness


    // depart_time.frame: 1641
    // depart_position: (64.6145, 72.8598, 0, 1641)
    // goal_ID: 3
    // self_ID: 48

    // await_drone drone;

    // // 限制 frame 在 1 到 20000 之间
    // drone.depart_time.frame = 1641;

    // // 限制 x, y, z 在 -100 到 100 之间
    // drone.depart_position.x = 64.6145;  // 随机 x 坐标
    // drone.depart_position.y = 72.8598;  // 随机 y 坐标
    // drone.depart_position.z = 0;  // 随机 z 坐标
    // drone.depart_position.frame = drone.depart_time.frame;  // 透传depart_time.frame
    // // 限制 goal_ID 和 self_ID 在 1 到 50 之间
    // drone.goal_ID = 3;
    // drone.self_ID = 48;
    // await_group.push_back(drone);


    for (size_t i = 0; i < total_quantity; i++)
    {   
        // 使用随机数生成器为 await_drone 的成员赋值
        await_drone drone;

        // 限制 frame 在 1 到 20000 之间
        drone.depart_time.frame = 1600 + std::rand() % (60*total_quantity);
        // drone.depart_time.frame = 1600;

        // 限制 x, y, z 在 -100 到 100 之间
        drone.depart_position.x = -100.0 + static_cast<double>(std::rand()) / RAND_MAX * 100.0;  // 随机 x 坐标
        drone.depart_position.y = -100.0 + static_cast<double>(std::rand()) / RAND_MAX * 100.0;  // 随机 y 坐标
        drone.depart_position.z = 0;  // 随机 z 坐标
        drone.depart_position.frame = drone.depart_time.frame;  // 透传depart_time.frame
        // 限制 goal_ID 和 self_ID 在 1 到 50 之间
        drone.goal_ID = 1 + std::rand() % 8;
        drone.self_ID = 10 + std::rand() % 50;

        // 打印本轮生成的 drone 对象信息
        // std::cout << "Drone " << i + 1 << " information:" << std::endl;
        // std::cout << "  depart_time.frame: " << drone.depart_time.frame << std::endl;
        // std::cout << "  depart_position: (" 
        //           << drone.depart_position.x << ", " 
        //           << drone.depart_position.y << ", " 
        //           << drone.depart_position.z << ")" << std::endl;
        // std::cout << "  goal_ID: " << drone.goal_ID << std::endl;
        // std::cout << "  self_ID: " << drone.self_ID << std::endl;
        // std::cout << std::endl;  // 空行用于区分每个 drone 的输出
        // 将 drone 添加到 await_group 容器中
        await_group.push_back(drone);
    }
    printf("Finifh init await_group\n");
}

static bool naturalOrderCompare(const std::string& a, const std::string& b) {
    std::regex re("(\\d+)");
    std::smatch match_a, match_b;

    std::regex_search(a, match_a, re);
    std::regex_search(b, match_b, re);

    if (!match_a.empty() && !match_b.empty()) {
        int num_a = std::stoi(match_a[0]);
        int num_b = std::stoi(match_b[0]);

        if (num_a != num_b) {
            return num_a < num_b;
        }
    }

    return a < b;
}

FileDescriptorManager::FileDescriptorManager(const std::string& path){
    if (!initialize(path)) {                           // 初始化路径
        std::cerr << "Failed to initialize file descriptor manager" << std::endl;
        return ;
    }
}

size_t FileDescriptorManager::initialize(const std::string& directory){
DIR* dir = opendir(directory.c_str());
    if (dir == nullptr) {
        std::cerr << "Failed to open directory: " << strerror(errno) << std::endl;
        return 0;
    }
    
    struct dirent* entry;

    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG) {
            std::string filePath = directory + "/" + entry->d_name;
            files.push_back(filePath);      /** 获得所有dac路径*/ 
        }
    }
    closedir(dir);
    capacity = files.size();

    // Sort files by name
    std::sort(files.begin(), files.end(), naturalOrderCompare);
    // std::cout << "After sort name:" << std::endl;
    // for (const auto& filePath : files) {
    //     std::cout << filePath << std::endl;
    // }

/** 动态开关fd时这里要注释掉 */
    int index = 0;
    for (const auto& filePath : files) {
        int fd = open(filePath.c_str(), O_RDWR);
        if (fd == -1) {
            std::cerr << "Failed to open file: " << filePath << " - " << strerror(errno) << std::endl;
            continue;
        }

        fileDescriptors_.push_back(fd);     /** 获得所有dac的FD*/ 
        index++;
    }
/** */

    std::cout << "The number of fileDescriptors_ is: " << capacity << std::endl; // 打印 fileDescriptors_ 的容量
    return capacity;

}

/** 随机访问任帧舞步*/
std::vector<set3d> FileDescriptorManager::read_eachFrame(const pps& frame) {

    std::vector<set3d> current_sequence;
    for (int i = 0; i < capacity; i++)
    {   
        // int fd = open(manager.files[i].c_str(), O_RDWR);
        // if (fd == -1) {      // 在这里做open close动态开关的
        // std::cerr << "Failed to open file: " << manager.files[i] << " - " << strerror(errno) << std::endl;
        // break;
        // }
        set3d position = get_eachPosition(/*fd*/i, frame);
        current_sequence.push_back(position);
        // if (close(fd) == -1) {
        //     std::cerr << "Failed to close file." << std::endl;
        // continue;
        // } 
    }
    // printf("finish one pushback frame\n");
    return current_sequence;
}

set3d FileDescriptorManager::read_eachFrame(const pps& frame, int goal_ID) {

        // int fd = open(manager.files[i].c_str(), O_RDWR);
        // if (fd == -1) {      // 在这里做open close动态开关的
        // std::cerr << "Failed to open file: " << manager.files[i] << " - " << strerror(errno) << std::endl;
        // break;
        // }
        set3d position = get_eachPosition(/*fd*/goal_ID, frame);

        // if (close(fd) == -1) {
        //     std::cerr << "Failed to close file." << std::endl;
        // continue;
        // } 

    // printf("finish one pushback frame\n");
    return position;
}

// 重载 read_eachFrame，接收 unsigned int 类型的参数
std::vector<set3d> FileDescriptorManager::read_eachFrame(unsigned int frame_num) {
    // 将 unsigned int 转换为 pps 类型
    pps frame(frame_num);
    // 调用原始的 read_eachFrame
    return read_eachFrame(frame);
}

set3d FileDescriptorManager::read_eachFrame(unsigned int frame_num, int goal_ID) {
    // 将 unsigned int 转换为 pps 类型
    pps frame(frame_num);
    // 调用原始的 read_eachFrame
    return read_eachFrame(frame, goal_ID);
}

set3d FileDescriptorManager::get_eachPosition(int index/*架次*/, const pps& frame/*, AES_ctx& ctx*/) {
        
        // printf("FD_num is %d \n", fileDescriptors_[index]);
        set3d Each_POS_and_RGB;
        off_t offset_origin = (frame.frame) * 22 + 46 + 1;                // 设置偏移
        // off_t offset = 46 + (frame - 1) * 22 + 1; // 46-byte header, 22-byte frame, +1 for stx

        if (index < 0 || index >= fileDescriptors_.size()) {
            std::cerr << "Drone index Not in the container" << std::endl;
            return Each_POS_and_RGB;}

        float pos_And_rgb[4];
        /** 读取块 互斥锁方式*/
        // {
        // std::unique_lock<std::mutex> lock(readSDcard_mutex);
        // ssize_t bytesRead = pread(fileDescriptors_[index], reinterpret_cast<char*>(pos_And_rgb), sizeof(pos_And_rgb), offset_origin);
        // if (bytesRead != sizeof(pos_And_rgb)) {std::cerr << "incomplete read!!" << std::endl;return Each_POS_and_RGB;}        // TODO 这里要加锁
        // }

        /** 读取块循环调用 pread方式*/

        ssize_t bytesRead = 0;
        // int countinner = 0;      // 用于计数拼接读取了多少次
        while (bytesRead < sizeof(pos_And_rgb)) {
        // countinner++;
        // printf("index: %d bytesRead :%d sizeof(pos_And_rgb):%d offset_origin:%d\n",index, bytesRead, sizeof(pos_And_rgb),offset_origin);
        ssize_t result = pread(fileDescriptors_[index], reinterpret_cast<char*>(&pos_And_rgb) + bytesRead, sizeof(pos_And_rgb) - bytesRead, offset_origin + bytesRead);
        
        if (result == -1) {
            std::cerr << "Error reading file: " << strerror(errno) << std::endl;
            break; // 错误时跳出循环，避免死循环
        }
        if (result == 0) {
            std::cerr << "End of file reached unexpectedly maybe frame outoff full dancestep?" << std::endl;
            break; // 读取到文件末尾时跳出循环
        }
            bytesRead += result;

    }
        // ssize_t result = pread(fileDescriptors_[index], reinterpret_cast<char*>(&pos_And_rgb) + bytesRead, sizeof(pos_And_rgb) - bytesRead, offset_origin + bytesRead);
        if (bytesRead != sizeof(pos_And_rgb)) {std::cerr << "incomplete read" << std::endl;return Each_POS_and_RGB;}        // TODO 这里要加锁
        // printf("bytesRead: %zd \n", bytesRead);

        /** 解码*/
        // AES_ECB_decrypt_buffer(&ctx, read_dance_buf/*read_dance_buf*/, sizeof(read_dance_buf)/*buf length*/);
        // AES_ECB_decrypt_buffer(&ctx, reinterpret_cast<unsigned char*>(pos)/*read_dance_buf*/, sizeof(pos)/*buf length*/);

        /** Debugging: Print raw bytes read*/
        // std::cout << "Raw bytes read:\n";
        // for (size_t i = 0; i < sizeof(pos); ++i) {
        //     printf("%02x ", reinterpret_cast<unsigned char*>(pos)[i]);
        // }std::cout << std::endl;

        /** Convert to vector for returning*/
        Each_POS_and_RGB.frame = frame.frame;       // 注意这里的帧偏移是从0序开始的
        Each_POS_and_RGB.x = pos_And_rgb[0];
        Each_POS_and_RGB.y = pos_And_rgb[1];
        Each_POS_and_RGB.z = pos_And_rgb[2];
        uint8_t RGBW[4];
        std::memcpy(RGBW, &pos_And_rgb[3], sizeof(float));

        Each_POS_and_RGB.R = RGBW[0];
        Each_POS_and_RGB.G = RGBW[1];
        Each_POS_and_RGB.B = RGBW[2];
        Each_POS_and_RGB.W = RGBW[3];
        
        // printf("num_x %f\n",Each_POS_and_RGB.x);
        // printf("num_y %f\n",Each_POS_and_RGB.y);
        // printf("num_z %f\n",Each_POS_and_RGB.z);
        // printf("num_frame %d\n",Each_POS_and_RGB.frame);
        return Each_POS_and_RGB;
}

/**
 float pos[3];
 float acc[3];
 uint16_t frame;
 uint16_t drone_id;
 uint8_t rgb[3];
 uint8_t res;
 uint8_t reserved[3];
 mavlink_auto_filling_dance_t;
*/


// void AlgorithmMng::inner_log() {
//     // 删除已有的log.txt文件
//     // std::remove("log.txt");

//     while (true) {
//         static int count = 0;
//         // std::lock_guard<std::mutex> lock(mtx_position); // 确保线程安全
//         printf("RCinfo: %d ID: %dx: %fy: %fz: %fvx: %fvy: %fvz: %fframe: %u",count, ID, virtual_posi.x, virtual_posi.y, virtual_posi.z, velocity.x, velocity.y, velocity.z, moment.frame);
//         printf("PRinfo: %d prx: %fpry: %fprz: %f",count, pos_predict.x, pos_predict.y, pos_predict.z);
//         printf("PLinfo: %d inversePlanning: %d  densimeter: %d  failPlanning_count: %d  bad_quadrantDrone_num: %d  endpoint_distance: %f  solution_time: %f",count, inversePlanning, densimeter, failPlanning_count, bad_quadrantDrone_num, endpoint_distance, solution_time);
//         printf("CYinfo: %d actualIndex: %d\n",count, queue.actualIndexx);
//         // printf("guiderUP: %d updated: %d\n",count, guider.updated);
//         // 使用 shell 命令将日志信息追加到文件末尾
//         // char command[512];
//         // snprintf(command, sizeof(command),
//         //          "echo \"Recevinfo Count_NUM: %d x: %f, y: %f, z: %f, frame: %u\" >> log.txt && echo \"Planninginfo %d \" >> log.txt",
//         //          count, virtual_posi.x, virtual_posi.y, virtual_posi.z, moment.frame);

//         // system(command);

//         count++;
//         std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 每500毫秒打印一次
//     }
// }

//  monitoring_data();

