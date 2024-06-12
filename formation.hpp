
#pragma once
#include <vector>

int start_frame = 25;				// 开始补位动作帧
double constraint_speed = 6;		// 速度约束
double collision_radius = 1.4;		// 避碰半径
int ALL_DRONE_NUM = 1934;			// 飞机总数

typedef struct vec3d	vec3d;
struct vec3d {
	double	x;
	double	y;
	double	z;
};

typedef struct set3d	set3d;
struct set3d {
	double	x;
	double	y;
	double	z;
    double  frame;
	// 默认构造函数
	set3d() : x(0), y(0), z(0) {}
    
    // 带参构造函数
    set3d(double x_val, double y_val = 0, double z_val = 0) : x(x_val), y(y_val), z(z_val) {}
};

class drone
{
	public:
		int drone_ID;
		std::vector<set3d> SET;
		int collision_num = 0;

		
		drone(int);
		
		// void 给出广播的若干点路径，找出这期间会发生轨迹交叉的飞机ID
		// void // 输入当前帧，查出当前位置
		// void export(int frame);			//TODO 给帧数输出对应位置
		// void get_radio(int frame, /*未来几帧轨迹*/);		// TODO 接收广播轨迹
		// void whether_collision();		// 判断轨迹是否重合
		// void avoid(/*应该return 一个指导向量,不一定要打卡位置*/);					// 规避机动策略
		// void speed_distance_calcu(); 视为满足位置约束前提,预先计算一下速度,来分为几帧分段
		// void 30frame 的轨迹插值函数
		// void 无人机随机方位飘动的函数
		// void 轨迹分段的函数
		// void 前导段策略函数,寻找最小闭包,方向策略是牵引向量,到达高度后横向插队
		// void 看看要不要三角化障碍块,这样有可能找不出解，增加避障难度，但是会增加避碰富余
		// void 仔细思考一个策略,每次计算的时刻,计算一个路径剔除一个路径,永远保持路径队列中有两三帧提前量是可用的,并且保持实时位置对齐帧,用一个反馈机制来证明可以跳入下一条规划好的路径(这里一定最好要引入反馈机制，没有的话就用超时机制)
		// void 可以在上升段分步计算，只遍历一次有层次交叉的飞机(算出大致到达时间,把所有有层级交叉的先过滤出来(全局)，再在这个小范围内去筛选会水平穿越的飞机)
		// TODO 最主要的时间开销是在寻机,比较舞步帧,计算范围和快速规划上 


};


