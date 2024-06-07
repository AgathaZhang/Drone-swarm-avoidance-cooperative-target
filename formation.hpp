
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
		drone(int);
		// void 给出广播的若干点路径，找出这期间会发生轨迹交叉的飞机ID
		// void // 输入当前帧，查出当前位置
		// void export(int frame);			//TODO 给帧数输出对应位置
		// void get_radio(int frame, /*未来几帧轨迹*/);		// TODO 接收广播轨迹
		// void whether_collision();		// 判断轨迹是否重合
		// void avoid(/*应该return 一个指导向量,不一定要打卡位置*/);					// 规避机动策略

};


