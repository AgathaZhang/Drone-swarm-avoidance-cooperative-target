#ifndef __FORMATION__
#define __FORMATION__
// #pragma once
#include <vector>

extern int start_frame;				// 开始补位动作帧
extern double constraint_speed;		// 速度约束
extern double collision_radius;		// 避碰半径
extern int ALL_DRONE_NUM;			// 飞机总数
// extern bool guide;

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

	// 运算符+重载
    vec3d operator+(const vec3d& other) const {
        return vec3d(x + other.x, y + other.y, z + other.z);
    }

    // 运算符-重载
    vec3d operator-(const vec3d& other) const {
        return vec3d(x - other.x, y - other.y, z - other.z);
    }

};

typedef struct vec2d vec2d;
struct vec2d {
    double x;
    double y;

    // 默认构造函数
    vec2d() : x(0), y(0) {}

    // 带参构造函数
    vec2d(double x_val, double y_val = 0) : x(x_val), y(y_val) {}

    // 运算符+重载
    vec2d operator+(const vec2d& other) const {
        return vec2d(x + other.x, y + other.y);
    }

    // 运算符-重载
    vec2d operator-(const vec2d& other) const {
        return vec2d(x - other.x, y - other.y);
    }
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

typedef struct Mint Mint;
struct Mint {
	int x;
	int y;
	int z;
};

typedef struct Mint2D Mint2D;
struct Mint2D {
	int x;
	int y;
};

typedef struct constraint
{
// pps start_frame;				// 开始补位动作帧
// vec3d init_position;			// 起始位置
double pixels;					// guide向量的步长分辨率
double framerate;				// 帧速率
// int calcu_times;				// 累计插值次数
// double elapsed_time;			// 累计时间
double constraint_speed;		// 合成最大速度约束
double collision_radius;		// 避碰半径
// int ALL_DRONE_NUM;				// 飞机总数
// SUCCESS_OR_NOT success;					// 是否补位成功
constraint()
        : /*start_frame{0},*/					// 约束里面不该出现变化的东西 不该出现起始时间
        //   init_position{0, 0, 0},
		  pixels{0.1},
		  framerate{30},
        //   calcu_times(0),
        //   elapsed_time(0.0),
          constraint_speed{3.0},
          collision_radius{0.2}
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
		// void 尽量让指导向量的频率与帧速率一致 可以采用归一化分配的思维 映射加插值 顺便做一个贝塞尔平滑
		// void 若下一时刻已经到来，上次的路径向量还没算完 这时候要锁止还是丢弃 丢弃：用最新的 设置一个cicalbuffer算到哪里存到哪里 最好不用全部算完
		// TODO 最主要的时间开销是在寻机,比较舞步帧,计算范围和快速规划上 
		// 不需要全部路径生成完毕 有多少生成就给多少 但是这个要在Astar库里面改
		// 每次生成轨迹只用判断 速度约束 之内的就可以 太远的不用放进来
		// 如果在途中 不亮灯 设置亮灯信号
		// 互斥锁考虑只锁当前坐标 不锁时间
		// 添加避碰半径0.3，可以调节
		// 远景规划 如果障碍过多 变线过于频繁 那就进入 超局部规划 或者 停顿 或者 激活函数
		// 可以试一下停在原处 自动规避 
		// 查找Z轴不满足速度约束的问题
		// mapping的时候要加入int16 尽量满足硬件的原始优化
		// 我这边给Unity 的Virtualdrone轨迹要按帧给 给planning的position要按2f(20cm)冗余量，实际中 往后算30frame
		// 1frame = 33ms  5frame = 165ms;



};

void Read_frame(std::vector<std::vector<set3d>>&);
void time_series_map(std::vector<std::vector<set3d>>&, const std::vector<drone>);
extern void socketCommunication(void);
extern std::vector<std::vector<set3d>> matrix;  
extern set3d view_matrix;  
extern bool isSorted;

#endif


