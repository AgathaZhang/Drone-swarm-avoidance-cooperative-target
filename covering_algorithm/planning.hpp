#ifndef __PLANNING__
#define __PLANNING__

#include "formation.hpp"
#include "AStar.hpp"
#include <mutex>

class Guide_vector {
public:
    std::vector<vec3d>& guide;
    pps moment;
    bool updated; // 是否已更新的标志

    // 构造函数
    Guide_vector(std::vector<vec3d>& guide = *(new std::vector<vec3d>()), const pps& moment = pps())
        : guide(guide), moment(moment) {updated = false;}

    // 更新函数
    void update(std::vector<vec3d>& new_guide, const pps& new_moment) {
        {std::lock_guard<std::mutex> lock(mtx);
        guide = new_guide;
        moment = new_moment;}
        {std::lock_guard<std::mutex> lock(updated_mtx);
        updated = true;}
    }
    // 读取函数
    std::pair<std::vector<vec3d>, pps> read() const {
        std::lock_guard<std::mutex> lock(mtx);
        return {guide, moment};
    }
    // 是否已到来新的更新
    bool is_Update_or_not(){
        std::lock_guard<std::mutex> lock(updated_mtx);
        if (updated == true) 
        {updated = false;
        return true;}
        
        else return false;
    }

private:
    mutable std::mutex mtx; // 保护更新操作的互斥锁
    
    std::mutex updated_mtx;
};

#endif