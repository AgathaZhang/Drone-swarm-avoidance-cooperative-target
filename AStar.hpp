/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>

namespace AStar
{
    struct Vec2i                                                            // 设置了二维向量 
    {
        int x, y;       

        bool operator == (const Vec2i& coordinates_);                       // 声明了重载函数
        friend Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_) {
            return{ left_.x + right_.x, left_.y + right_.y };
        }
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;    // 表示一个返回值为uint 的函数格式
    using CoordinateList = std::vector<Vec2i>;                      // CoordinateList坐标点

    struct Node             // 每个点的 过去g(n) 未来h(n)
    {
        uint G, H;          // 代价为实数
        Vec2i coordinates;  // 坐标
        Node *parent;       // 父节点

        Node(Vec2i coord_, Node *parent_ = nullptr);      // 初始化操作
        uint getScore();    // 代价和
    };

    using NodeSet = std::vector<Node*>;                     // Node指针集

    class Generator     // 生产者类
    {
        bool detectCollision(Vec2i coordinates_);                       // 碰撞检测
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);      // 在集合中找, coordinates参数1,2相等返回指针
        void releaseNodes(NodeSet& nodes_);                             // 删除整个NodeSet,线性时间复杂度，vector后面的元素全部移动一次

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);                            // 设置世界边界
        void setDiagonalMovement(bool enable_);                         // 启用对角线搜寻
        void setHeuristic(HeuristicFunction heuristic_);                // 设置启发函数
        CoordinateList findPath(Vec2i source_, Vec2i target_);          // 寻路,给出源和目标,输出一个坐标容器,即路径
        void addCollision(Vec2i coordinates_);                          // 添加墙
        void removeCollision(Vec2i coordinates_);                       // 移除墙
        void clearCollisions();                                         // 清空墙
        void _bezier_curve();/*贝塞尔平滑 未定义,将就加速度约束*/

    private:
        HeuristicFunction heuristic;                // 格式 uint(Vec2i, Vec2i)
        CoordinateList direction, walls;            // "方位""墙"均为std::vector<Vec2i>类型
        Vec2i worldSize;
        uint directions;
    };

    class Heuristic         // 启发类
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static uint euclidean(Vec2i source_, Vec2i target_);        // 源位置和目标间的欧氏距离,返回一个标量代价
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
