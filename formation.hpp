#ifndef __FORMATION__
#define __FORMATION__
// #pragma once
#include <vector>

extern int start_frame;				// 开始补位动作帧
extern double constraint_speed;		// 速度约束
extern double collision_radius;		// 避碰半径
extern int ALL_DRONE_NUM;			// 飞机总数
extern bool guide;

typedef enum{
	YES = 1,		// 补位完成
	NO = 0,			// 补位失败
	ACTIVE= 2,		// 正在补位
}SUCCESS_OR_NOT;


typedef struct pps pps;
struct pps{
	// double time;
	unsigned int frame;
};

typedef struct vec3d vec3d;		//指导结果向量
struct vec3d {
	double	x;
	double	y;
	double	z;
	// 默认构造函数
	vec3d() : x(0), y(0), z(0) {}
    // 带参构造函数
    vec3d(double x_val, double y_val = 0, double z_val = 0) : x(x_val), y(y_val), z(z_val) {}
};

typedef struct set3d set3d;
struct set3d {
	double	x;
	double	y;
	double	z;
    unsigned int frame;
	// 默认构造函数
	set3d() : x(0), y(0), z(0) {}
    // 带参构造函数
   set3d(double x_val, double y_val = 0, double z_val = 0, unsigned int frame_val = 0)
        : x(x_val), y(y_val), z(z_val), frame(frame_val) {}
};

typedef struct constraint
{
// pps start_frame;				// 开始补位动作帧
// vec3d init_position;			// 起始位置
double pixels;
// int calcu_times;				// 累计插值次数
// double elapsed_time;			// 累计时间
double constraint_speed;		// 速度约束
// double collision_radius;		// 避碰半径
// int ALL_DRONE_NUM;				// 飞机总数
// SUCCESS_OR_NOT success;					// 是否补位成功
constraint()
        : /*start_frame{0},*/					// 约束里面不该出现变化的东西 不该出现起始时间
        //   init_position{0, 0, 0},
		  pixels{0.2},
        //   calcu_times(0),
        //   elapsed_time(0.0),
          constraint_speed{3.0}
        //   collision_radius(0.0),
        //   ALL_DRONE_NUM(0),
        //   success(2) 
		{}

}constraint;

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
		// void 前导段策略函数,寻找最小闭包,方向策略是牵引向量,到达高度后横向插队 (目前策略是直接上升)
		// void 看看要不要三角化障碍块,这样有可能找不出解，增加避障难度，但是会增加避碰富余
		// void 仔细思考一个策略,每次计算的时刻,计算一个路径剔除一个路径,永远保持路径队列中有两三帧提前量是可用的,并且保持实时位置对齐帧,用一个反馈机制来证明可以跳入下一条规划好的路径(这里一定最好要引入反馈机制，没有的话就用超时机制)
		// void 可以在上升段分步计算，只遍历一次有层次交叉的飞机(算出大致到达时间,把所有有层级交叉的先过滤出来(全局)，再在这个小范围内去筛选会水平穿越的飞机)
		// void 加上实时位置积分，系统纯滞后，计算误差累积
		// void 上升段计算闭包最短直线距离
		// void 因为30frame/s,动得很慢，所以可以做帧合并以匹配给出vec3d指导向量的速率
		// 搜索空间优化：间歇性全局计算，从上次选的里面找10次 时间上选取前1帧后3帧
		// void 加入激活函数 回退路径甚至是原地等待，随舞步缓慢移动
		// void 有可能飞行表演已经结束了还没补到位 这个情况要大致估算一下是否可以释放出发
		// TODO 最主要的时间开销是在寻机,比较舞步帧,计算范围和快速规划上 


};

void Read_frame(std::vector<std::vector<set3d>>&);
void time_series_map(std::vector<std::vector<set3d>>&, const std::vector<drone>);
extern void socketCommunication(void);
extern std::vector<std::vector<set3d>> matrix;  
extern set3d view_matrix;  
extern bool isSorted;

#endif


