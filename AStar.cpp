#include "AStar.hpp"
#include <algorithm>
#include <math.h>

using namespace std::placeholders;


AStar::Node::Node(Vec3i coordinates_, Node *parent_) {
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore() {
    return G + H;
}

AStar::Generator::Generator() {
    setDiagonalMovement(false);                 // 是否八方移动
    setHeuristic(&Heuristic::manhattan);        // 设置启发函数为曼哈顿距离
    direction = {                               // 26个方向
        { 0, 1, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { -1, 0, 0 },
        { 0, 0, 1 }, { 0, 0, -1 },      //0-5

        { 1, 1, 0 }, { 1, -1, 0 }, { -1, 1, 0 }, { -1, -1, 0 },
        { 1, 0, 1 }, { 1, 0, -1 }, { -1, 0, 1 }, { -1, 0, -1 },
        { 0, 1, 1 }, { 0, 1, -1 }, { 0, -1, 1 }, { 0, -1, -1 },         // 6-17

        { 1, 1, 1 }, { 1, 1, -1 }, { 1, -1, 1 }, { 1, -1, -1 },         // 18-25
        { -1, 1, 1 }, { -1, 1, -1 }, { -1, -1, 1 }, { -1, -1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec3i worldSize_) {
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_) {
    directions = (enable_ ? 26 : 6);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_) {
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec3i coordinates_) {
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec3i coordinates_) {
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions() {
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec3i source_, Vec3i target_) {
    Node *current = nullptr;
    NodeSet openSet, closedSet;             // Node*指针集
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));   // 加入源节点 到openSet

    while (!openSet.empty()) {              // 不为空执行 保证路径唯一性 这里看要不要考虑多解情况
        auto current_it = openSet.begin();  // current_it = 第一个元素的迭代器  //指针
        current = *current_it;              // 让当前指针指向openSet首 
/** 第一个循环用比较优劣 */
        for (auto it = openSet.begin(); it != openSet.end(); it++) {        // 遍历openSet 选出最小代价方向
            auto node = *it;
            if (node->getScore() <= current->getScore()) {      // 与当前节点比较分数 有可能出现分数相等的情况 按比较的顺序选择咯
                current = node;                                 // 保存对象 （选代价小的留）
                current_it = it;                                // 保存迭代器
            }
        }

        if (current->coordinates == target_) {                  // 达到目标
            break;
        }

        closedSet.push_back(current);                                       // 放入closeSet
        openSet.erase(current_it);                                          // 从openSet擦除
/** 第二个循环用于拓展生成 */
        for (uint i = 0; i < directions; ++i) {
            Vec3i newCoordinates(current->coordinates + direction[i]);      // 探索一个方位
            if (detectCollision(newCoordinates) ||  // 碰到边界和障碍返回true
                findNodeOnList(closedSet, newCoordinates)) { // 已走路径不放入，nullptr视为false,
                continue;
            }

            // uint totalCost = current->G + 10;  // 简化为统一代价
            // uint totalCost = current->G + ((i < 6) ? 17 : ((i < 18) ? 14 : 10));     // 斜边为主
            uint totalCost = current->G + ((i < 6) ? 10 : ((i < 18) ? 14 : 17));     // 平直为主
            // uint totalCost = current->G + ((i < 6) ? 10 : 14);     // 平直为主

            Node *successor = findNodeOnList(openSet, newCoordinates);      // 如果没找到 要找n！次   是否重复放入openSet 这只是个安全机制
            if (successor == nullptr) {                                     // 没重复放在openSet集
                successor = new Node(newCoordinates, current);              // 拓展新节点 添加父节点
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);  
                openSet.push_back(successor);                               
            } else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);
    // printf("yes");
    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec3i coordinates_) {
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_) {
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec3i coordinates_) {
    if (coordinates_.x <= (-worldSize.x) || coordinates_.x >= worldSize.x ||
        coordinates_.y <= (-worldSize.y)|| coordinates_.y >= worldSize.y ||
        coordinates_.z <= (-worldSize.z) || coordinates_.z >= worldSize.z ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec3i AStar::Heuristic::getDelta(Vec3i source_, Vec3i target_) {
    return { abs(source_.x - target_.x),  abs(source_.y - target_.y),  abs(source_.z - target_.z) };
}

AStar::uint AStar::Heuristic::manhattan(Vec3i source_, Vec3i target_) {
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y + delta.z));
}

AStar::uint AStar::Heuristic::euclidean(Vec3i source_, Vec3i target_) {
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(50 * sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec3i source_, Vec3i target_) {
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y + delta.z) + (-6) * std::min({delta.x, delta.y, delta.z});
}