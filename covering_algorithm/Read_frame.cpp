
#include "formation.hpp"

int start_frame = 25;				// 开始补位动作帧
double constraint_speed = 6;		// 速度约束
double collision_radius = 1.4;		// 避碰半径
int ALL_DRONE_NUM = 1934;			// 飞机总数

size_t FileDescriptorManager::initialize(const std::string& directory) {
        DIR* dir = opendir(directory.c_str());
        if (dir == nullptr) {
            std::cerr << "Failed to open directory: " << strerror(errno) << std::endl;
            return 0;
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
        capacity = fileDescriptors_.size();
        std::cout << "The number of fileDescriptors_ is: " << capacity << std::endl; // 打印 fileDescriptors_ 的容量
        return capacity;
    }

FileDescriptorManager::~FileDescriptorManager() {
        for (auto& entry : fileDescriptors_) {
            close(entry.second);
        }
    }

set3d FileDescriptorManager::getFramePosition(int index/*架次*/, const pps& frame/*, AES_ctx& ctx*/) {
        
        // printf("FD_num is %d \n", fileDescriptors_[index]);
        static pps innner_frame = frame;
        set3d position;
        off_t offset_origin = (innner_frame.frame) * 22 + 46 + 1;                // 设置偏移
        // off_t offset_origin = 0;
        // off_t offset = 46 + (frame - 1) * 22 + 1; // 46-byte header, 22-byte frame, +1 for stx

        if (fileDescriptors_.find(index) == fileDescriptors_.end()) {
            std::cerr << "Drone index Not in the container" << std::endl;
            return position;}

        /** 读取块*/
        float pos[3];
        ssize_t bytesRead = pread(fileDescriptors_[index], reinterpret_cast<char*>(pos), sizeof(pos), offset_origin);
        // if (bytesRead != sizeof(pos)) {std::cerr << "incomplete read" << std::endl;return position;}

        // /** 打印源数据*/printf("------------------------------------------- origin hex data\n"); 
        // for (int i = 0; i < 1024; ++i) {
        //     printf("%02x ", pos[i]);
        //     if ((i + 1) % 16 == 0) {
        //         printf(" \n"); // 每行打印16个字节
        // }}printf("\n\n\n");

        /** 解码*/
        // AES_ECB_decrypt_buffer(&ctx, read_dance_buf/*read_dance_buf*/, sizeof(read_dance_buf)/*buf length*/);
        // AES_ECB_decrypt_buffer(&ctx, reinterpret_cast<unsigned char*>(pos)/*read_dance_buf*/, sizeof(pos)/*buf length*/);

        /** Debugging: Print raw bytes read*/
        std::cout << "Raw bytes read:\n";
        for (size_t i = 0; i < sizeof(pos); ++i) {
            printf("%02x ", reinterpret_cast<unsigned char*>(pos)[i]);
        }std::cout << std::endl;

        // Convert to vector for returning
        position.x = pos[0];
        position.y = pos[1];
        position.z = pos[2];
        position.frame = innner_frame.frame;
        printf("num_x %f\n",position.x);
        printf("num_y %f\n",position.y);
        printf("num_z %f\n",position.z);
        printf("num_frame %d\n",position.frame);
        return position;
    }

void FileDescriptorManager::listFiles() const {
        for (size_t i = 0; i < fileNames_.size(); ++i) {
            std::cout << i << ": " << fileNames_[i] << std::endl;
        }
    }

CircularQueue::CircularQueue(size_t size) : size_(size), front_(0), tail_(0), count_(0) {
        queue_.resize(size_);   
    }

bool CircularQueue::enqueue(const std::vector<set3d> & item) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return count_ < size_; });
        queue_[tail_] = item;
        tail_ = (tail_ + 1) % size_;
        count_++;
        cv_.notify_all();
        return true;
    }

bool CircularQueue::dequeue(std::vector<set3d> & item) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return count_ > 0; });
        item = queue_[front_];
        front_ = (front_ + 1) % size_;
        count_--;
        cv_.notify_all();
        return true;
    }

bool CircularQueue::isFull() const {
        std::unique_lock<std::mutex> lock(mutex_);
        return count_ == size_;
    }

bool CircularQueue::isEmpty() const {
        std::unique_lock<std::mutex> lock(mutex_);
        return count_ == 0;
    }


drone::drone(int id) {                                                      // Drone构造函数
		drone_ID = id;
}


// const uint8_t encript_key[16] = {0x02, 0x05, 0x00, 0x08, 0x01, 0x07, 0x00, 0x01, 0x01, 0x09, 0x09, 0x01, 0x00, 0x07, 0x02, 0x04};
// AES_ctx ctx;
// AES_init_ctx(&ctx, encript_key);

std::vector<set3d> Read_frame(const pps& frame, FileDescriptorManager manager) {

    std::vector<set3d> current_sequence;

    for (size_t i = 0; i < manager.capacity; i++)
    {
        set3d position = manager.getFramePosition(i, frame);
        current_sequence.push_back(position);
    }
    
    return current_sequence;

}


void loadInCycque(const pps& first_moment, CircularQueue& queue) {                  // 循环队列装载线程 根据first_moment加上帧号

    FileDescriptorManager manager;                                                  // 初始化FD管理器对象
    // if (!manager.initialize("/mnt/sdcard/Dac_data")) {                           // 初始化板载路径
    if (!manager.initialize("../Dac_data")) {
        std::cerr << "Failed to initialize file descriptor manager" << std::endl;
        return ;
    }
    manager.listFiles();

    while (true)
    {
        std::vector<set3d> current_sequence = Read_frame(first_moment, manager);
        queue.enqueue(current_sequence);
    }

}

// 消耗线程函数
// void processData(CircularQueue& queue) {
//     while (true) {
//         DataBlock data;
//         queue.dequeue(data);

//         // 处理数据
//         std::cout << "Processing data at offset " << data.offset
//                   << " with coordinates (" << data.x << ", " << data.y << ", " << data.z << ")" << std::endl;
//     }
// }

