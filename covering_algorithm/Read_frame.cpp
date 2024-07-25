#include "formation.hpp"
#include <thread>

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
        set3d position;
        off_t offset_origin = (frame.frame) * 22 + 46 + 1;                // 设置偏移
        // off_t offset_origin = 0;
        // off_t offset = 46 + (frame - 1) * 22 + 1; // 46-byte header, 22-byte frame, +1 for stx

        if (fileDescriptors_.find(index) == fileDescriptors_.end()) {
            std::cerr << "Drone index Not in the container" << std::endl;
            return position;}

        /** 读取块*/
        float pos[3];
        ssize_t bytesRead = pread(fileDescriptors_[index], reinterpret_cast<char*>(pos), sizeof(pos), offset_origin);
        // printf("bytesRead: %zd \n", bytesRead);
        if (bytesRead != sizeof(pos)) {std::cerr << "incomplete read" << std::endl;return position;}        // TODO 这里要加锁

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
        // std::cout << "Raw bytes read:\n";
        // for (size_t i = 0; i < sizeof(pos); ++i) {
        //     printf("%02x ", reinterpret_cast<unsigned char*>(pos)[i]);
        // }std::cout << std::endl;

        /** Convert to vector for returning*/
        position.x = pos[0];
        position.y = pos[1];
        position.z = pos[2];
        position.frame = frame.frame;       // 注意这里的帧偏移是从0序开始的
        // printf("num_x %f\n",position.x);
        // printf("num_y %f\n",position.y);
        // printf("num_z %f\n",position.z);
        // printf("num_frame %d\n",position.frame);
        return position;
    }

void FileDescriptorManager::listFiles() const {
        for (size_t i = 0; i < fileNames_.size(); ++i) {
            std::cout << i << ": " << fileNames_[i] << std::endl;
        }
    }

CircularQueue::CircularQueue(size_t size) : size_(size), front_(0), tail_(0), count_(0), atomicity(1) {
    queue_.resize(size_);
}

bool CircularQueue::enqueue(const std::vector<set3d>& item) {
    std::unique_lock<std::mutex> lock(mutex_enqueue_dequeue_);
    cv_.wait(lock, [this] { return count_ < size_; });
    queue_[tail_] = item;
    tail_ = (tail_ + 1) % size_;
    count_++;
    cv_.notify_all();
    return true;
}

bool CircularQueue::dequeue(std::vector<set3d>& item/*取的最后一帧实体*/, size_t moment_sequence/*当前到哪一帧了*/) {
    std::unique_lock<std::mutex> lock(mutex_enqueue_dequeue_);
    cv_.wait(lock, [this] { return count_ > 0 && atomicity == 1; });

    while (true) {
        // 解锁以调用invoking
        // lock.unlock();
        set3d tempItem = invoking(0/* 表示头位置*/, 1/*表示该系列选一个FD*/);       // TODO 这里的索引有点问题 这里可以用invoking随意访问
        // lock.lock();

        if (tempItem.frame < moment_sequence) {
            front_ = (front_ + 1) % size_;
            count_--;
        } else {
            break;
        }
    }

    item = queue_[front_];
    cv_.notify_all();
    return true;
}

// 重载的dequeue方法
bool CircularQueue::dequeue(size_t moment_sequence) {
    std::vector<set3d> item; // 临时变量
    return dequeue(item, moment_sequence); // 调用已有版本的dequeue
}

bool CircularQueue::isFull() const {
    std::unique_lock<std::mutex> lock(mutex_enqueue_dequeue_);
    return count_ == size_;
}

bool CircularQueue::isEmpty() const {
    std::unique_lock<std::mutex> lock(mutex_enqueue_dequeue_);
    return count_ == 0;
}

set3d CircularQueue::invoking(size_t sequence/*时间帧*/, size_t index/*架次*/) {
    // std::unique_lock<std::mutex> lock(mutex_dequeue_invoking_);
    size_t actualIndex = (front_ + sequence) % size_;
    if (actualIndex >= count_) {
        std::cerr << "Error: Sequence index out of range. actualIndex: " << actualIndex << ", count_: " << count_ << std::endl;
        throw std::out_of_range("Sequence index out of range");
    }
    if (index >= queue_[actualIndex].size()) {
        std::cerr << "Error: Inner vector index out of range. index: " << index << ", inner vector size: " << queue_[actualIndex].size() << std::endl;
        throw std::out_of_range("Inner vector index out of range");
    }
    return queue_[actualIndex][index];
}


// const uint8_t encript_key[16] = {0x02, 0x05, 0x00, 0x08, 0x01, 0x07, 0x00, 0x01, 0x01, 0x09, 0x09, 0x01, 0x00, 0x07, 0x02, 0x04};
// AES_ctx ctx;
// AES_init_ctx(&ctx, encript_key);

std::vector<set3d> Read_frame(const pps& frame, FileDescriptorManager& manager) {

    std::vector<set3d> current_sequence;
    // set3d position = manager.getFramePosition(4, frame);  // 单架fd测试
    for (size_t i = 0; i < manager.capacity; i++)
    {
        set3d position = manager.getFramePosition(i, frame);
        current_sequence.push_back(position);
    }
    
    return current_sequence;

}

void consumeInCycque(const pps& first_moment, CircularQueue& queue) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));        // 给点时间让buffer装到超前的帧 防止死锁
    while (true) {
       queue.dequeue(first_moment.frame);
    }
}

void loadInCycque(const pps& first_moment, CircularQueue& queue) {                  // 循环队列装载线程 根据first_moment加上帧号

    FileDescriptorManager manager;                                                  // 初始化FD管理器对象
    if (!manager.initialize("/mnt/sdcard/Dac_data100")) {                           // 初始化板载路径
    // if (!manager.initialize("../Dac_data")) {
        std::cerr << "Failed to initialize file descriptor manager" << std::endl;
        return ;
    }
    // manager.listFiles();


    static pps inner_frame = first_moment;
    while (true)
    {   
        // std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
        std::vector<set3d> current_sequence = Read_frame(inner_frame, manager);    // TODO manager改为引用传递 读出单个序列
        // if (!queue.isFull())// 不需要额外检查了
        {   
            // printf("load frame now :%d \n",current_sequence[1].frame);
            queue.enqueue(current_sequence);                                        // 每个序列添加到队列
            // queue.dequeue(first_moment.frame);
        }
        // queue.prt_count();    // buffer的容量监控打印
        inner_frame.frame++;
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// 消耗线程函数
// void consumeInCycque(CircularQueue& queue) {
//     while (true) {
//         std::vector<set3d> data;
//         queue.dequeue(data, 10000);
//         auto view = data.size();printf("size current sequence: %d \n", view);
//         set3d single_data = data[5];
//         // 处理数据
//         std::cout << "Processing data at offset " << single_data.frame
//                   << " with coordinates (" << single_data.x << ", " << single_data.y << ", " << single_data.z << ")" << std::endl;
//         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//     }
// }



