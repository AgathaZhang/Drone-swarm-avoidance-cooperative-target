/**
  ******************************************************************************
  * @author  Zhang weizhi
  * @date    5-Jun-2024
  * @brief   Drone-swarm-avoidance-cooperative-target
  ******************************************************************************

 * Function simpleAtof 不依赖库函数转换浮点为double
 * Function read_calibration_table 读取存在table.c中的字符串数组 table.c用于存校准表

 */

int start_frame = 25;			// 开始帧
double constraint_speed = 6;		// 速度约束
// 避碰半径

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
};

class drone
{
	public:
		int drone_ID;
		std::vector<set3d> SET;
		drone(int);
		// void export(int frame);			//TODO 给帧数输出对应位置
		// void get_radio(int frame, /*未来几帧轨迹*/);		// TODO 接收广播轨迹
		// void whether_collision();		// 判断轨迹是否重合
		// void avoid(/*应该return 一个指导向量,不一定要打卡位置*/);					// 规避机动策略

};

