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
    double  timestamp;
};

class drone
{
    public:
    std::vector<set3d> SET;
};
