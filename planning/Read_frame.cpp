#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "formation.hpp"
#include "aes.hpp"

// int start_frame = 25;				// 开始补位动作帧
// double constraint_speed = 6;		// 速度约束
// double collision_radius = 1.4;		// 避碰半径
// int ALL_DRONE_NUM = 1934;			// 飞机总数

#include <iostream>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <unordered_map>
#include <cstring>
#include <arpa/inet.h>
// extern AES_ctx ctx;
// extern void AES_ECB_decrypt_buffer(struct AES_ctx* ctx, uint8_t* buf,uint32_t length);

class FileDescriptorManager {
public:
    
    bool initialize(const std::string& directory) {
        DIR* dir = opendir(directory.c_str());
        if (dir == nullptr) {
            std::cerr << "Failed to open directory: " << strerror(errno) << std::endl;
            return false;
        }

        struct dirent* entry;
        int index = 0;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_type == DT_REG) {
                std::string filePath = directory + "/" + entry->d_name;
                int fd = open(filePath.c_str(), O_RDWR);
                if (fd == -1) {
                    std::cerr << "Failed to open file: " << filePath << " - " << strerror(errno) << std::endl;
                    continue;
                }
                fileDescriptors_[index] = fd;
                fileNames_.push_back(filePath);
                index++;
            }
        }
        closedir(dir);
        size_t capacity = fileDescriptors_.size();

        // 打印 fileDescriptors_ 的容量
        std::cout << "The capacity of fileDescriptors_ is: " << capacity << std::endl;

        return true;
    }

    ~FileDescriptorManager() {
        for (auto& entry : fileDescriptors_) {
            close(entry.second);
        }
    }

    ssize_t readFromFile(int index, char* buffer, size_t count, off_t offset) {
    if (fileDescriptors_.find(index) == fileDescriptors_.end()) {
        std::cerr << "Invalid file descriptor index" << std::endl;
        return -1;
    }
    // return read(fileDescriptors_[index], buffer, count);
    return pread(fileDescriptors_[index], buffer, count, offset);
}
//     // float 转十进制
//     float hexStringToFloat(const std::string& hexStr) {
//     // 将十六进制字符串转换为 uint32_t 类型的整数
//     uint32_t intValue;
//     std::stringstream ss;
//     ss << std::hex << hexStr;
//     ss >> intValue;

//     // 将 uint32_t 类型的整数解释为 float 类型的浮点数
//     float floatValue;
//     std::memcpy(&floatValue, &intValue, sizeof(float));

//     return floatValue;
// }

    std::vector<float> getFramePosition(int index/*文件描述符序号*/, unsigned int frame/*帧序数*/, AES_ctx ctx) {

        std::vector<float> position(3, 0.0f);
        off_t offset_origin = (frame) * 22 + 46;
        // off_t offset = 46 + (frame - 1) * 22 + 1; // 46-byte header, 22-byte frame, +1 for stx

        if (fileDescriptors_.find(index) == fileDescriptors_.end()) {
            std::cerr << "Invalid file descriptor index" << std::endl;
            return position;
        }

        // Read the pos[3] values (3 floats)
        printf("FD_num is %d \n", fileDescriptors_[index]);
        uint8_t read_dance_buf[1024];
        float pos[3];
        ssize_t bytesRead = readFromFile(index, reinterpret_cast<char*>(read_dance_buf), sizeof(read_dance_buf), offset_origin);
        
        for (int i = 0; i < 1024; ++i) {
        printf("%02x ", read_dance_buf[i]);
        if ((i + 1) % 16 == 0) {
            printf(" \n"); // 每行打印16个字节
        }
    }
        // ssize_t bytesRead = readFromFile(index, reinterpret_cast<char*>(pos), sizeof(pos), offset);
        // if (bytesRead != sizeof(read_dance_buf)) {
        // // if (bytesRead != sizeof(pos)) {
        //     std::cerr << "Failed to read position data or incomplete read" << std::endl;
        //     return position;
        // }
        AES_ECB_decrypt_buffer(&ctx, read_dance_buf/*read_dance_buf*/, sizeof(read_dance_buf)/*buf length*/);
        printf("------------------------------------------- \n\n\n"); 

        for (int i = 0; i < 1024; ++i) {
        printf("%02x ", read_dance_buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n"); // 每行打印16个字节
        }
    }
        // AES_ECB_decrypt_buffer(&ctx, reinterpret_cast<unsigned char*>(pos)/*read_dance_buf*/, sizeof(pos)/*buf length*/);
        memcpy(pos, read_dance_buf + 1, sizeof(float) * 3);

        // Debugging: Print raw bytes read
        std::cout << "Raw bytes read:\n";
        for (size_t i = 0; i < sizeof(pos); ++i) {
            printf("%02x ", reinterpret_cast<unsigned char*>(pos)[i]);

        }
        std::cout << std::endl;

        // Convert to vector for returning
        position[0] = pos[0];
        position[1] = pos[1];
        position[2] = pos[2];
        
        return position;
    }
    // ssize_t writeToFile(int index, const char* buffer, size_t count) {
    //     if (fileDescriptors_.find(index) == fileDescriptors_.end()) {
    //         std::cerr << "Invalid file descriptor index" << std::endl;
    //         return -1;
    //     }
    //     return write(fileDescriptors_[index], buffer, count);
    // }

    void listFiles() const {
        for (size_t i = 0; i < fileNames_.size(); ++i) {
            std::cout << i << ": " << fileNames_[i] << std::endl;
        }
    }

private:
    std::unordered_map<int, int> fileDescriptors_;
    std::vector<std::string> fileNames_;
};

// ---------------------------------------------------------------------------------------------
drone::drone(int id) {                                                      // Drone构造函数
		drone_ID = id;
}


const uint8_t encript_key[16] = {0x02, 0x05, 0x00, 0x08, 0x01, 0x07, 0x00, 0x01, 0x01, 0x09, 0x09, 0x01, 0x00, 0x07, 0x02, 0x04};


void Read_frame(const unsigned int frame/*帧序*/) {
    AES_ctx ctx;
    AES_init_ctx(&ctx, encript_key);
    FileDescriptorManager manager;
    
    // if (!manager.initialize("/mnt/sdcard/Dac_data")) {                           // 初始化
    if (!manager.initialize("../Dac_data")) {
        std::cerr << "Failed to initialize file descriptor manager" << std::endl;
        return ;
    }
    manager.listFiles();

    char buffer[12];
    std::vector<float> position = manager.getFramePosition(0/*fd架次*/, frame, ctx);
    if (!position.empty()) {
        std::cout << "Frame " << frame << " Position: ("
                  << position[0] << ", " << position[1] << ", " << position[2] << ")" << std::endl;
    }
    // ssize_t bytesRead = manager.readFromFile(0, buffer, sizeof(buffer) - 1, 24);
    // if (bytesRead > 0) {
    //     buffer[bytesRead] = '\0';
    //     std::cout << "Read from file: " << buffer << std::endl;
    // }

}
