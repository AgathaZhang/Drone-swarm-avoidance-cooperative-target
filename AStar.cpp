#include "AStar.hpp"
#include <algorithm>
#include <math.h>

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);                 // 是否八方移动
    setHeuristic(&Heuristic::manhattan);        // 设置启发函数为曼哈顿距离
    direction = {                               // 8个方向
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}
/* key func 寻路函数*/
AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;             // Node指针集
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));   // 加入源节点 到openSet

    while (!openSet.empty()) {              // 不为空执行 保证路径唯一性 这里看要不要考虑多解情况
        auto current_it = openSet.begin();  // current_it = 第一个元素的指针
        current = *current_it;              // 让当前指针指向openSet首          

        for (auto it = openSet.begin(); it != openSet.end(); it++) {        // 遍历openSet
            auto node = *it;
            if (node->getScore() <= current->getScore()) {      // 与当前节点比较分数
                current = node;                                 // 保存对象 （选代价小的留）
                current_it = it;                                // 保存迭代器
            }
        }

        if (current->coordinates == target_) {                  // TODO达到目标 无解机制需要考虑浮点数需要考虑
            break;
        }

        closedSet.push_back(current);                                       // 放入closeSet
        openSet.erase(current_it);                                          // 从openSet擦除

        for (uint i = 0; i < directions; ++i/*from 1-4*/) {
            Vec2i newCoordinates(current->coordinates + direction[i]);      // TODO 探索一个方位 这里看是否设置一个步长 此处调用构造函数
            if (detectCollision(newCoordinates) ||  /*碰到边界和障碍返回ture*/
                findNodeOnList(closedSet, newCoordinates)) {                // 已走路径，nullptr视为false,
                continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);              // 已走代价 横着是10 斜着是14

            Node *successor = findNodeOnList(openSet, newCoordinates);      // 如果没找到找n！次   这只是个安全机制
            if (successor == nullptr) {                                     // 没在openSet集
                successor = new Node(newCoordinates, current);              // 拓展新节点添加父节点
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);  
                openSet.push_back(successor);                               
            }
            else if (totalCost < successor->G) {
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

    return path;
}

/*  在集合中找到了，返回这个Node @param nodes_ 集合,    coordinates_ 当前坐标*/

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)                          // TODO 修改以针对负象限的情况
{
    if (coordinates_.x <= (-worldSize.x) || coordinates_.x >= worldSize.x ||
        coordinates_.y <= (-worldSize.y) || coordinates_.y >= worldSize.y ||                      // TODO 实数化,进某个范数球即为碰撞
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {       // 找到重叠,返回一个指向找到元素的迭代器,否则返回 walls.end() 思考：只有一个值的时候直接指向walls.end()
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));            // 10为线性系数 static_cast<uint>类型转换 引入了量化误差,但节省了计算开销
}                                                                                      // TODO 考虑要不要加入随机震荡机制来激活局部极值的情况

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

// TODO 加入位置约束失步之后的避碰，启用摄像头传感器，视情况加速穿越