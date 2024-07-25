#ifndef HG_MATH_H_
#define HG_MATH_H_
#include <math.h>
#include <string.h>
#include <stdarg.h>//传递不定长参数 

#ifndef FLT_EPSILON
#define FLT_EPSILON             (1.1920929e-07f)  /* 1E-5 */
#endif
#ifndef M_PI_D	
#define M_PI_D	                (3.141592653589793238462643383279502884)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD              (0.017453292519943295769236907684886f)	
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG              (57.295779513082320876798154814105f)	
#endif

#ifndef CONSTANTS_ONE_G
#define CONSTANTS_ONE_G		    (9.80665f)	   /* m/s^2		*/
#endif

#ifndef M_PI_F
#define M_PI_F                  (3.1415926f)	
#endif

template <typename T>
class Matrix3;

template <typename T>
class Vector3
{
	public:
	//默认构造
	Vector3() {x = 0; y = 0; z = 0;};
	//指定参数构造
	Vector3(T _x, T _y, T _z):x (_x), y (_y), z (_z){};
	//修改元素值
	inline void set(T _x, T _y, T _z){x = _x; y = _y; z = _z;};
	inline void set_zero(void) {x = 0; y = 0; z = 0;}
	
	template <typename K>
	inline Vector3<K> cast(void) const
	{
		return Vector3<K>(K(this->x), K(this->y), K(this->z));
	}
	//重载+运算符
	inline Vector3<T> operator+(const Vector3<T> &v) const
	{
	    return  Vector3<T>(this->x + v.x, this->y + v.y, this->z + v.z);
	}
	//重载+=运算符
	inline void operator+=(const Vector3<T> &v) 
	{
	    this->x += v.x;
		this->y += v.y;
		this->z += v.z;
	}
	//重载负号运算符
	inline Vector3<T> operator-(void) const
	{
	    return  Vector3<T>(-this->x , -this->y, -this->z);
	}
	//重载-运算符
	inline Vector3<T> operator-(const Vector3<T> &v) const
	{
	    return  Vector3<T>(this->x - v.x, this->y - v.y,this->z - v.z);
	}
	//重载-=运算符
	inline void operator-=(const Vector3<T> &v)
	{
	    this->x -= v.x;
		this->y -= v.y;
		this->z -= v.z;
	}
	//重载*运算符
	inline T operator*(const Vector3<T> &v) const
	{
	    return  this->x * v.x + this->y * v.y + this->z * v.z;
	}
	//重载*运算符
	inline Vector3<T> operator*(const T &k) const
	{
	    return  Vector3<T>(this->x * k, this->y * k,this->z * k);
	}
	//重载*=运算符
	inline void operator*=(const T &k) 
	{
	    this->x *= k;
		this->y *= k;
		this->z *= k;
	}   
	//重载/运算符
	inline Vector3<T> operator/(const T &k) const
	{
	    return  Vector3<T>(this->x / k, this->y / k,this->z / k);
	}
	//重载/=运算符
	inline void operator/=(const T &k)
	{
	   	this->x /= k;
		this->y /= k;
		this->z /= k;
	}
	
	//重载()运算符
	inline T operator()(unsigned char n) const
	{
	   	switch(n)
		{
			case 0:
				return this->x;
		    case 1:
				return this->y;
			default:
				return this->z;
		}
	}
	
	inline T operator[](unsigned char n) const
	{
	   	switch(n)
		{
			case 0:
				return this->x;
		    case 1:
				return this->y;
			default:
				return this->z;
		}
	}
	
	//重载()运算符
	inline void operator()(T x, T y, T z) 
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
	//向量叉乘
	inline Vector3<T> operator%(const Vector3<T> &v)const
	{
	    Vector3<T> vector;
	    vector.x = this->y * v.z - this->z * v.y;
	    vector.y = this->z * v.x - this->x * v.z;
	    vector.z = this->x * v.y - this->y * v.x;
		return vector;
	}
	//向量叉乘
	inline Vector3<T> cross(const Vector3<T> &v) const
	{
	    Vector3<T> vector;
	    vector.x = this->y * v.z - this->z * v.y;
	    vector.y = this->z * v.x - this->x * v.z;
	    vector.z = this->x * v.y - this->y * v.x;
		return vector;
	}
	
	inline void normalize(void) 	
	{
		T norm = sqrtf(this->x * this->x + this->y * this->y + this->z * this->z);
		if (fabs(norm) > FLT_EPSILON)
		{
		    this->x /= norm; this->y /= norm;this->z /= norm;
		}	
	}

    inline T norm(void)	const
	{
		return sqrtf(this->x * this->x + this->y * this->y + this->z * this->z);
	}
	
	Vector3<T> constrain(T low, T up)
	{
		Vector3<T> vector;

		vector.x = this->x > up ? up:(this->x < low ? low : this->x);
		vector.y = this->y > up ? up:(this->y < low ? low : this->y);
		vector.z = this->z > up ? up:(this->z < low ? low : this->z);
		return vector;	
	}
	
	inline void zero(void)	
	{
		this->x = T(0.0f);
	    this->y = T(0.0f);
	    this->z = T(0.0f);
	}

    //计算向量的反对称阵(行向量)
	inline Matrix3<T> shew_matrix(void)
	{
		return Matrix3<T>((T)0.0f, -z,     y,
				          z ,     (T)0.0f, -x,
				          -y,      x ,     (T)0.0f) ;
	}

    T x, y, z;
};




template <typename T>
class Quat
{
	public:
	//默认构造	
	Quat() {q0 = 1; q1 = 0; q2 = 0; q3 = 0;};
	//指定参数构造
	Quat(T _q0, T _q1, T _q2, T _q3):q0(_q0), q1(_q1), q2(_q2), q3(_q3){};
	//四元素构造
	Quat(const Quat &_q):q0(_q.q0), q1(_q.q1), q2(_q.q2), q3(_q.q3){};
		
	Quat(const Vector3<T> &euler)
    {
		double cosPhi_2   = cos(double(euler.x) / 2.0);
		double sinPhi_2   = sin(double(euler.x) / 2.0);
		double cosTheta_2 = cos(double(euler.y) / 2.0);
		double sinTheta_2 = sin(double(euler.y) / 2.0);
		double cosPsi_2   = cos(double(euler.z) / 2.0);
		double sinPsi_2   = sin(double(euler.z) / 2.0);

		/* operations executed in double to avoid loss of precision through
		 * consecutive multiplications. Result stored as float.
		 */
		q0 = static_cast<float>(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
		q1 = static_cast<float>(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
		q2 = static_cast<float>(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
		q3 = static_cast<float>(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
		
		T norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 /= norm;
		q1 /= norm;
		q2 /= norm;
		q3 /= norm;	
	}
	//修改元素值	
	inline void set(T _q0, T _q1, T _q2, T _q3){q0 = _q0; q1 = _q1; q2 = _q3; q3 = _q3;};

	//重载负号，四元素取反
	inline Quat<T> operator-(void)
	{
		return Quat<T>(-this->q0, -this->q1, -this->q2, -this->q3);
	}
	//四元素乘法
	inline Quat<T> operator*(const Quat<T> &_q)
	{
		Quat<T> q;
		q.q0 = this->q0 * _q.q0 - this->q1 * _q.q1 - this->q2 * _q.q2 - this->q3 * _q.q3;
		q.q1 = this->q1 * _q.q0 + this->q0 * _q.q1 - this->q3 * _q.q2 + this->q2 * _q.q3;
		q.q2 = this->q2 * _q.q0 + this->q3 * _q.q1 + this->q0 * _q.q2 - this->q1 * _q.q3;
		q.q3 = this->q3 * _q.q0 - this->q2 * _q.q1 + this->q1 * _q.q2 + this->q0 * _q.q3;
		
		T norm = sqrtf(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
		q.q0 /= norm;
		q.q1 /= norm;
		q.q2 /= norm;
		q.q3 /= norm;
		return q;
	}
	//四元素乘法
	inline void operator*=(const Quat<T> &_q)
	{
		Quat<T> q;
		q.q0 = this->q0 * _q.q0 - this->q1 * _q.q1 - this->q2 * _q.q2 - this->q3 * _q.q3;
		q.q1 = this->q1 * _q.q0 + this->q0 * _q.q1 - this->q3 * _q.q2 + this->q2 * _q.q3;
		q.q2 = this->q2 * _q.q0 + this->q3 * _q.q1 + this->q0 * _q.q2 - this->q1 * _q.q3;
		q.q3 = this->q3 * _q.q0 - this->q2 * _q.q1 + this->q1 * _q.q2 + this->q0 * _q.q3;
		
		T norm = sqrtf(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
		this->q0 = q.q0/norm;
		this->q1 = q.q1/norm;
		this->q2 = q.q2/norm;
		this->q3 = q.q3/norm;
	}
	//四元素乘常数
	inline Quat<T> operator*(const T &k)
	{
		Quat<T> q;
		q.q0 = this->q0 * k;
		q.q1 = this->q1 * k;
		q.q2 = this->q2 * k;
		q.q3 = this->q3 * k;
		return q;
	}
	//四元素乘常数
	inline void operator*=(const T &k)
	{
		this->q0 *= k;
		this->q1 *= k;
		this->q2 *= k;
		this->q3 *= k;
	}
	//重载/运算符，这里k需要外部检查是否为0
	inline Quat<T> operator/(const T &k)
	{
		Quat<T> q;
		if (fabs(k) > FLT_EPSILON)
		{
			q.q0 = this->q0/k;
			q.q1 = this->q1/k;
			q.q2 = this->q2/k;
			q.q3 = this->q3/k;
		}
		else
		{
			q = *this;
		}

		return q;
	}
	//重载/=运算符，这里k需要外部检查是否为0
	inline Quat<T> operator/=(const T &k)
	{
		if (fabs(k) > FLT_EPSILON)
		{
			this->q0 /= k;
			this->q1 /= k;
			this->q2 /= k;
			this->q3 /= k;
		}
	}

	//四元素的微分
	inline Quat<T> derivative1(const Vector3<T>& v)
	{
		Quat<T> dq;
		dq.q0 = 0.5f * (0 - v.x * this->q1 - v.y * this->q2 - v.z * this->q3);
		dq.q1 = 0.5f * (v.x * this->q0 + 0 + v.z * this->q2 - v.y * this->q3);
		dq.q2 = 0.5f * (v.y * this->q0 - v.z * this->q1 + 0 + v.x * this->q3);
		dq.q3 = 0.5f * (v.z * this->q1 + v.y * this->q1 + 0 - v.x * this->q3);
	
	    return dq;
	}
	
	//四元素归一化
	inline void normalize(void)
	{
		T norm = sqrtf(this->q0 * this->q0 + this->q1 * this->q1 + this->q2 * this->q2 + this->q3 * this->q3);
		
		if (fabs(norm) > FLT_EPSILON)
		{
			this->q0 /= norm; this->q1 /= norm; this->q2 /= norm; this->q3 /= norm;
		} 
	}
	
	//四元素模值
	inline T norm(void) const
	{
		return sqrtf(this->q0 * this->q0 + this->q1 * this->q1 + this->q2 * this->q2 + this->q3 * this->q3);
	}
	
	//四元素更新公式Qn = Q(n-1) * Q(n-1)(n), 其中Q(n-1)(n) 实现如下
	inline Quat<T> expq(const Vector3<T>& u)
	{
		const T tol = T(0.2);           // ensures an error < 10^-10
		const T c2 = T(1.0 / 2.0);      // 1 / 2!
		const T c3 = T(1.0 / 6.0);      // 1 / 3!
		const T c4 = T(1.0 / 24.0);     // 1 / 4!
		const T c5 = T(1.0 / 120.0);    // 1 / 5!
		const T c6 = T(1.0 / 720.0);    // 1 / 6!
		const T c7 = T(1.0 / 5040.0);   // 1 / 7!
        //这里注意u是常量，norm必须常函数
		T u_norm = u.norm();
		T sinc_u, cos_u;

		if (u_norm < tol) 
		{
			T u2 = u_norm * u_norm;
			T u4 = u2 * u2;
			T u6 = u4 * u2;

			// compute the first 4 terms of the Taylor serie
			sinc_u = T(1.0) - u2 * c3 + u4 * c5 - u6 * c7;
			cos_u = T(1.0) - u2 * c2 + u4 * c4 - u6 * c6;
		} 
		else
		{
			sinc_u = T(sin(u_norm) / u_norm);
			cos_u = T(cos(u_norm));
		}

		Vector3<T> v =  u * sinc_u;
		return Quat<T>(cos_u, v.x, v.y, v.z);
	}
	//四元素转欧拉角
	inline Vector3<T> to_euler() const
	{
		T phi, theta, psi;
		
        //方法二
		Matrix3<T> mat = Matrix3<T>(*this);
		
		theta = asin(-mat(2, 0));

		if ((fabs(theta - T(M_PI_D / 2))) < T(1.0e-3)) 
		{
			phi = 0;
			psi = atan2(mat(1, 2), mat(0, 2));
		} 
		else if ((fabs(theta + T(M_PI_D / 2))) < T(1.0e-3)) 
	    {
			phi = 0;
			psi = atan2(-mat(1, 2), -mat(0, 2));
		} 
		else 
		{
			phi = atan2(mat(2, 1), mat(2, 2));
			psi = atan2(mat(1, 0), mat(0, 0));
		}
        //方法二
        // theta = asin(2.0f * (q2 * q3 + q0 * q1));

        // if (abs(2.0f *(q0 * q1 + q2 * q3)) <= 0.999999f)
        // {
        //     phi = -atan2(2.0f *(q1 * q3 - q0 * q2), q0 * q0 -q1*q1-q2*q2 + q3*q3);

        //     psi = -atan2(2.0f *(q1 * q2 - q0 * q3), q0 * q0 -q1*q1+q2*q2 - q3*q3);
        // }
        // else
        // {
        //     phi = atan2(2.0f *(q1 * q3 + q0 * q2), q0 * q0 +q1*q1-q2*q2 - q3*q3);

        //     psi = 0.0f;
        // }
        // //方法三
        // phi   = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
        // theta = asinf(2.0f * (q0 * q2 - q1 * q3));
        // psi   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));


		return Vector3<T>(phi, theta, psi);
	}

    inline void from_dcm(Matrix3<T> &dcm)
	{
		//这里的处理还需要优化一下
		q0 = 0.5f * sqrt(1 + dcm(0, 0) + dcm(1, 1) + dcm(2, 2));
        q1 = (dcm(2, 1) - dcm(1, 2)) / (4 * q0);
        q2 = (dcm(0, 2) - dcm(2, 0)) / (4 * q0);
        q3 = (dcm(1, 0) - dcm(0, 1)) / (4 * q0);
	}

    inline Matrix3<T> to_dcm(void) const
	{
		Matrix3<T> dcm;
        T a = q0;
        T b = q1;
        T c = q2;
        T d = q3;
        T aSq = a*a;
        T bSq = b*b;
        T cSq = c*c;
        T dSq = d*d;
        dcm(0, 0) = aSq + bSq - cSq - dSq;
        dcm(0, 1) = 2 * (b * c - a * d);
        dcm(0, 2) = 2 * (a * c + b * d);
        dcm(1, 0) = 2 * (b * c + a * d);
        dcm(1, 1) = aSq - bSq + cSq - dSq;
        dcm(1, 2) = 2 * (c * d - a * b);
        dcm(2, 0) = 2 * (b * d - a * c);
        dcm(2, 1) = 2 * (a * b + c * d);
        dcm(2, 2) = aSq - bSq - cSq + dSq;
		
		return dcm;
	}		
	
    T q0, q1, q2, q3;
};



template <typename T>
class Matrix3
{
	public:
	//默认构造		
	Matrix3(){memset(m, 0, sizeof(T));};
	//四元素构造
	Matrix3(const Quat<T> &_q)
	{
		//检查四元素是否为0外面做
		T data = _q.norm();
		//归一化
		Quat<T> norm;
		norm.q0 = _q.q0 / data;
		norm.q1 = _q.q1 / data;
		norm.q2 = _q.q2 / data;
		norm.q3 = _q.q3 / data;
				
	    m[0][0] = 1 -2 * norm.q2 * norm.q2 -2 * norm.q3 * norm.q3;
		m[0][1] = 2 * norm.q1 * norm.q2 - 2 * norm.q0 * norm.q3; 
		m[0][2] = 2 * norm.q1 * norm.q3 + 2 * norm.q0 * norm.q2;
		m[1][0] = 2 * norm.q1 * norm.q2 + 2 * norm.q0 * norm.q3;
		m[1][1] = 1 -2 * norm.q1 * norm.q1 -2 * norm.q3 * norm.q3;
		m[1][2] = 2 * norm.q2 * norm.q3 - 2 * norm.q0 * norm.q1;
		m[2][0] = 2 * norm.q1 * norm.q3 - 2 * norm.q0 * norm.q2; 
		m[2][1] = 2 * norm.q2 * norm.q3 + 2 * norm.q0 * norm.q1; 
		m[2][2] = 1 -2 * norm.q1 * norm.q1 -2 * norm.q2 * norm.q2;
	}
	//欧拉角构建旋转矩阵(NED321次序)
	Matrix3(const Vector3<T> &euler)
	{
		Matrix3<T> &dcm = *this;
		T cosPhi = T(cos(euler.x));
		T sinPhi = T(sin(euler.x));
		T cosThe = T(cos(euler.y));
		T sinThe = T(sin(euler.y));
		T cosPsi = T(cos(euler.z));
		T sinPsi = T(sin(euler.z));

		dcm.m[0][0] = cosThe * cosPsi;
		dcm.m[0][1] = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
		dcm.m[0][2] = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

		dcm.m[1][0] = cosThe * sinPsi;
		dcm.m[1][1] = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
		dcm.m[1][2] = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

		dcm.m[2][0] = -sinThe;
		dcm.m[2][1] = sinPhi * cosThe;
		dcm.m[2][2] = cosPhi * cosThe;
	}
	
    //矩阵元素指定参数
	Matrix3(const T _q0, const T _q1, const T _q2, 
		    const T _q3, const T _q4, const T _q5, 
			const T _q6, const T _q7, const T _q8)
    {
	    m[0][0] = _q0; m[0][1] = _q1;m[0][2] = _q2;
		m[1][0] = _q3; m[1][1] = _q4;m[1][2] = _q5;
		m[2][0] = _q6; m[2][1] = _q7;m[2][2] = _q8;
    }
	
    //矩阵元素指定参数
	inline void operator()(const T _q0, const T _q1, const T _q2, 
		                   const T _q3, const T _q4, const T _q5, 
			               const T _q6, const T _q7, const T _q8)
    {
	    m[0][0] = _q0; m[0][1] = _q1;m[0][2] = _q2;
		m[1][0] = _q3; m[1][1] = _q4;m[1][2] = _q5;
		m[2][0] = _q6; m[2][1] = _q7;m[2][2] = _q8;
    }
	
	inline void identity(void)
	{
		this->zero();
		
		this->m[0][0] = T(1.0f);
		this->m[1][1] = T(1.0f);
		this->m[2][2] = T(1.0f);
	}
	
	template <typename K>
	inline Matrix3<K> cast(void)
	{
		Matrix3<K> ret;
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
			    ret[i][j] = this->m[i][i];
			}
		}
	
	    return ret;
	}
	
	//矩阵元素指定参数
	inline void set(const T _q0, const T _q1, const T _q2, 
		            const T _q3, const T _q4, const T _q5, 
			        const T _q6, const T _q7, const T _q8)
    {
	    m[0][0] = _q0; m[0][1] = _q1;m[0][2] = _q2;
		m[1][0] = _q3; m[1][1] = _q4;m[1][2] = _q5;
		m[2][0] = _q6; m[2][1] = _q7;m[2][2] = _q8;
    }
	
	inline void set_row(int row, const Vector3<T> &vec)
    {
		if (row > 2) row = 2;
		if (row < 0) row = 0;
	    m[row][0] = vec.x; 
		m[row][1] = vec.y; 
		m[row][2] = vec.z; 
    }
	
	inline void set_col(int row, const Vector3<T> &vec)
    {
		if (row > 2) row = 2;
		if (row < 0) row = 0;
	    m[0][row] = vec.x; 
		m[1][row] = vec.y; 
		m[2][row] = vec.z; 
    }
		
	//矩阵清零
	inline void zero()
    {
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
			    m[i][j] = (T)0.0f;
			}
		}
    }
	
	//矩阵加法
	inline Matrix3<T> operator+(const Matrix3<T> &matrix) 
	{
		Matrix3<T> mat;
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
			    mat.m[i][j] = m[i][j] + matrix.m[i][j];
			}
		}

	    return mat;
	}
	
	//矩阵加法
	inline void operator+=(const Matrix3<T> &matrix) 
	{
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
			    m[i][j] += matrix.m[i][j];
			}
		}
	}
	
	//矩阵减法
	inline Matrix3<T> operator-(const Matrix3<T> &matrix) 
	{
		Matrix3<T> mat;
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
			    mat.m[i][j] = m[i][j] - matrix.m[i][j];
			}
		}

	    return mat;
	}
	//矩阵取负
	inline Matrix3<T> operator-(void) const
	{
		Matrix3<T> mat;
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
			    mat.m[i][j] = -m[i][j];
			}
		}
	    return mat;
	}
	
	//矩阵加法
	inline void operator-=(const Matrix3<T> &matrix) 
	{
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
			    m[i][j] -= matrix.m[i][j];
			}
		}
	}
	
	//矩阵乘以向量
	inline Vector3<T> operator*(const Vector3<T>& v) const
	{
		Vector3<T> vector;	
		vector.x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z;
		vector.y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z;
		vector.z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z;
	    return vector;
	}
	
    //矩阵乘以矩阵
	inline Matrix3<T> operator*(Matrix3<T> &mat)
	{
        Matrix3<T> temp;
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
				temp.m[i][j] = m[i][0] * mat.m[0][j] + m[i][1] * mat.m[1][j] + m[i][2] * mat.m[2][j];
			}
		}	
        return temp;
	}
	
	//矩阵乘以常数
	inline Matrix3<T> operator*(const T &k)
	{
		Matrix3<T> mat;
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
				mat.m[i][j] = m[i][j] * k;
			}
		}																			  
	    return mat;
	}
	//矩阵乘以常数
	inline void operator*=(const T &k)
	{
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
				this->m[i][j] *= k;
			}
		}																			  
	}
	
	//重载()符号
	inline T& operator()(int row, int col) 
	{
		if (row >2) row = 2;
		if (col >2) col = 2;
	    return m[row][col];
	}
	
	//三维矩阵转置
	inline Matrix3<T> transpose(void)
	{
		Matrix3<T> mat;
		for (uint8_t i = 0; i < 3; i++)
		{
			for (uint8_t j = 0; j < 3; j++)
			{
				mat.m[i][j] = m[j][i];
			}
		}
	    return mat;
	}
	
	//三维向量构建三维对角阵
	inline Matrix3<T> diag(const Vector3<T> &v)
	{
		//默认构造会清零
		Matrix3<T> mat;

		mat.m[0][0] = v.x;
		mat.m[1][1] = v.y;
		mat.m[2][2] = v.z;

		return mat;
	}
	
	//三维矩阵求逆
	inline Matrix3<T> inverse(void)
	{
		//默认构造会清零
		Matrix3<T> mat;
		T det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
		        m[0][1] * (m[1][0] * m[2][2] - m[2][0] * m[1][2]) +
		        m[0][2] * (m[1][0] * m[2][1] - m[2][0] * m[1][1]);
		
		if (fabs(det) > FLT_EPSILON)
		{
			mat.m[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) / det;
			mat.m[1][0] = -(m[2][0] * m[1][2] - m[1][0] * m[2][2]) / det;
			mat.m[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) / det;
			
            mat.m[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) / det;
			mat.m[1][1] = (m[0][0] * m[2][2] - m[2][0] * m[0][2]) / det;
			mat.m[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) / det;
			
            mat.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det;	
			mat.m[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) / det;	
			mat.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det;	
		}
		else
		{
			//否则返回原值
			mat = *this;
		}
		return mat;
	}
	
    T m[3][3];
};


template <typename T, uint8_t N>
class VectorN
{
public:
    // trivial ctor
    inline VectorN<T, N>() 
    {
        memset(_v, 0, sizeof(T)*N);
    }

    // vector ctor
    inline VectorN<T,N>(const T *v) 
    {
        memcpy(_v, v, sizeof(T)*N);
    }
	
	//不定长赋值
	void set(T n, ...) 
	{	
		va_list argptr;  
      
        va_start(argptr, n);
		
		for(uint8_t i = 0; i < n ; i++)
		{  
            _v[i] = va_arg(argptr, T);  
        } 
        va_end(argptr);		
    }
	inline T operator()(uint8_t i) 
    {
        return _v[i];
    }
    
    inline T & operator[](uint8_t i) 
    {
        return _v[i];
    }
	

    inline const T & operator[](uint8_t i) const
    {
        return _v[i];
    }

    // test for equality
    bool operator ==(const VectorN<T,N> &v) const 
    {
        for (uint8_t i=0; i<N; i++) 
        {
            if (_v[i] != v[i]) 
                return false;
        }
        return true;
    }

    // zero the vector
    inline void zero()
    {
        memset(_v, 0, sizeof(T)*N);
    }

    // negation
    VectorN<T,N> operator -(void) const 
    {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) 
		{
            v2[i] = - _v[i];
        }
        return v2;
    }

    // addition
    VectorN<T,N> operator +(const VectorN<T,N> &v) const 
    {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) 
        {
            v2[i] = _v[i] + v[i];
        }
        return v2;
    }

    // subtraction
    VectorN<T,N> operator -(const VectorN<T,N> &v) const 
    {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) 
		{
            v2[i] = _v[i] - v[i];
        }
        return v2;
    }

    // uniform scaling
    VectorN<T,N> operator *(const T num) const 
    {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) 
		{
            v2[i] = _v[i] * num;
        }
        return v2;
    }

    // uniform scaling
    VectorN<T,N> operator  /(const T num) const 
    {
        VectorN<T,N> v2;


        for (uint8_t i = 0; i < N; i++) 
        {
            v2[i] = _v[i] / num;
        }
        return v2;
    }

    // addition
    VectorN<T,N> &operator +=(const VectorN<T,N> &v)
    {
        for (uint8_t i=0; i<N; i++) 
        {
            _v[i] += v[i];
        }
        return *this;
    }

    // subtraction
    VectorN<T,N> &operator -=(const VectorN<T,N> &v) 
    {
        for (uint8_t i=0; i<N; i++) 
        {
            _v[i] -= v[i];
        }
        return *this;
    }

    // uniform scaling
    VectorN<T,N> &operator *=(const T num) 
    {
        for (uint8_t i=0; i<N; i++) 
        {
            _v[i] *= num;
        }
        return *this;
    }

    // uniform scaling
    VectorN<T,N> &operator /=(const T num) 
    {
        for (uint8_t i=0; i<N; i++) 
        {
            _v[i] /= num;
        }
        return *this;
    }

    // dot product
    T operator *(const VectorN<T,N> &v) const 
    {
        float ret = 0;
        for (uint8_t i=0; i<N; i++) 
        {
            ret += _v[i] * v._v[i];
        }
        return ret;
    }
    

	
	T norm(void)
	{
		T temp = 0;
		for (int i = 0; i < N; i++)
		{
			temp += _v[i] * _v[i];  
		} 
		return sqrt(temp);
	}


    T _v[N];
};



template<typename T, uint8_t M, uint8_t N>
class Matrix 
{
    public:
	//默认构造函数
	Matrix<T, M, N>(void) 
	{
		row = M;
		col = N;
		memset(data, 0, M * N * sizeof(T));        
	}
	

	/**
	 * 由二维下标索引矩阵元素，可读写
	 * @param i :矩阵i+1行 j :矩阵j+1行
	 * @return 矩阵元素的引用
	 */
	T& operator ()(uint8_t i, uint8_t j)
	{
	    return data[i][j];
	}
	

	/**
	 * 由行向量构造对角阵
	 * @param m1:M * 1向量
	 * @return 对角阵
	 */
	void diagonal(Matrix<T, M, 1> m1)
	{
		uint8_t i = 0;
		
		for(i = 0; i < M; i++)
		{
			data[i][i] = m1.data[i][0] * m1.data[i][0];
	
		} 
	}
	
	
	/**
	 * 将3*3矩阵拷贝到当前矩阵的(rows，cols)到(rows + 3，cols + 3)列处
	 * @param rows :rows+1行，cols:cols+1列
	 * @param d：待拷贝的矩阵
	 */
	template <uint8_t H, uint8_t W>
	void block(uint8_t rows, uint8_t cols, Matrix3<T> d)
	{
		T * temp_ptr = &data[rows][cols];
		
		*temp_ptr++ = d.a.x;
		*temp_ptr++ = d.a.y;
		*temp_ptr++ = d.a.z;
		temp_ptr    =  &data[rows + 1][cols];
		*temp_ptr++ = d.b.x;
		*temp_ptr++ = d.b.y;
		*temp_ptr++ = d.b.z;
		temp_ptr    =  &data[rows + 2][cols];
		*temp_ptr++ = d.c.x;
		*temp_ptr++ = d.c.y;
		*temp_ptr++ = d.c.z;	
	} 
	
	
	/**
	 * 将3维向量拷贝到当前矩阵的rows+1行，cols+1列处，行存储
	 * @param rows:ows+1行，cols:cols+1列
	 * @param d：待拷贝的向量
	 */
    template <uint8_t H, uint8_t W>
	void block(uint8_t rows, uint8_t cols, Vector3<T> d)
	{
		T * temp_ptr = &data[rows][cols];
		  
		*temp_ptr++ = d.x;
		*temp_ptr++ = d.y;
		*temp_ptr++ = d.z;	
	}
	
	/**
	 * 将P*Q维矩阵中(0, 0)到(H, W)部分拷贝到当前矩阵的起始处
	 * @param H :H+1行，W:W+1列
	 * @param d：待拷贝的向量
	 */
	template <uint8_t P, uint8_t Q>
	void block(uint8_t H, uint8_t W, Matrix<T, P, Q> d)
	{
		T *ptr;
		
		for (uint8_t i = 0; i < H; i++)
		{
			ptr = (T *)(d.data[i]);
			
			for (uint8_t j = 0; j < W; j++)
			{
				data[i][j] = *ptr++;
			}
		}
	} 
	
	/**
	 * 将M*N矩阵所有元素清零
	 */
	void  zero()
	{
	    uint8_t i,j;
		for(i = 0; i < M; i++)
		{
			for(j = 0;j < N; j++)
			{
			    data[i][j] = 0;
			}
		}
	}
	
	/**
	 * 将M*N矩阵对角元素置1
	 */
	void ones(void)  
	{
		uint8_t n = N > M ? M : N;
		
		for (uint8_t i = 0; i < n ; i++)
		{
			data[i][i] = 1;
		}
	}
	
	/**
	 * 当前M*N大小矩阵加上M*N大小矩阵B
	 */
	Matrix<T, M, N> operator +(Matrix<T, M, N> B)
	{
        Matrix<T, M, N> temp;
		
	    uint8_t i,j;
		for (i = 0; i < M; i++)
		{
			for (j = 0; j < N; j++)
			{
			    temp.data[i][j] = data[i][j] + B.data[i][j];
			}
		}
		return temp;
	}
	
	/**
	 * 当前M*N大小矩阵减去M*N大小矩阵B
	 */
	Matrix<T, M, N> operator -(Matrix<T, M, N> B)
	{
        Matrix<T, M, N> temp;
		
	    uint8_t i,j;
		for(i = 0; i < M; i++)
		{
			for(j = 0;j < N; j++)
			{
			    temp.data[i][j] = data[i][j] - B.data[i][j];
			}
		}
		return temp;
	}
	
	/**
	 * 当前M*N大小矩阵乘以常数num
	 */
	Matrix<T, M, N> operator *(T num)
	{
        Matrix<T, M, N> temp;
		
	    uint8_t i,j;
		for(i = 0; i < M; i++)
		{
			for(j = 0;j < N; j++)
			{
			    temp.data[i][j] = data[i][j] * num;
			}
		}
		return temp;
	}
	
	/**
	 * 当前M*N大小矩阵乘以N*K大小矩阵mat
	 */
	template <uint8_t K>
	Matrix<T, M, 	K> operator *(const Matrix<T, N, K> &mat)
	{
		uint8_t i = 0, j = 0, p = 0;
		
		Matrix<T, M, K> temp;
		
		T s = 0;
		
		for(i = 0; i < M; i++)
		{
			for(j = 0;j < N; j++)
			{
				s = data[i][j];
				
				for(p = 0; p < K; p++)
				{
					temp.data[i][p] += s * mat.data[j][p];
				}
			}
		}

		return temp;  
	}
	
	/**
	 * 当前M*N大小矩阵乘以三维列向量v
	 */
	VectorN<T, M> operator *(const Vector3<T> v)
	{
		VectorN<T, M> temp;
		
		uint8_t i;
		
		for (i = 0; i < M; i++)
		{
			temp._v[i] = data[i][0] * v.x + data[i][1] * v.y + data[i][2] * v.z;
		}
		return temp;  
	}

	/**
	 * 当前M*N大小矩阵乘以N维列向量v
	 */
	VectorN<T, M> operator *(const VectorN<T, N> v)
	{
		VectorN<T, M> temp;
		
		uint8_t i, j;
		
		for (i = 0; i < M; i++)
		{
			for (j = 0; j < N; j++)
			{
			    temp._v[i] += data[i][j] * v._v[j];
			}
		}
		return temp;  
	}
		
	
	/**
	 * 当前M*N大小矩阵转置
	 */
	Matrix<T, N, 	M> transpose()
	{
		Matrix<T, N, M> temp;
		
		uint8_t i,j, k;
		
		for(i = 0; i < M; i++)
		{
			for(j = 0;j < N; j++)
			{
				temp.data[j][i] = data[i][j]; 	
			}
		}
		return temp;
	}
	
	
	
	/**
	 * 当前M*N大小矩阵采用高斯消元法求逆
	 */
	Matrix<T, N, 	M> Gaussian_elimination(T arr[N][N])
	{
		Matrix<T, N, 	M> result;
		int i, j, k;

		char un_inv_flag = 0;
		T W[N][2 * N];
		T tem_1, tem_2, tem_3;
	 
		/// 矩阵扩维
		for (i = 0; i < N; i++)
		{
			memcpy(&W[i][0], &arr[i][0], N * sizeof(T));
			
			memset(&W[i][N], 0, N * sizeof(T));
			/// 扩充矩阵单位化
			W[i][N + i] = 1.0;	
		}
	 
		for (i = 0; i < N; i++)
		{
			/// 
			if((abs(W[i][i])) < 1e-15)
			{ 
				for (j = i + 1; j < N; j++)
				{
					if((abs(W[i][i])) < 1e-15) 
					{
						break;
					}
						
				}
				if(j == N)
				{
					/// 不可逆
					un_inv_flag = 1;
					break;
				}
				//
				for(k = 0; k < 2 * N; k++)
				{
					W[i][k] += W[j][k];
				}
			}
	 

			tem_1 = W[i][i];
			for (j = 0; j < 2 * N; j++)
			{
				W[i][j] = W[i][j] / tem_1;
			}
	 
			for (j = i+1; j < N; j++)
			{
				tem_2 = W[j][i];
				for (k = i; k < 2 * N; k++)
				{
					W[j][k] = W[j][k] - tem_2 * W[i][k];
				}
			}
		}
	 
		for (i = N - 1; i >= 0; i--)
		{
			for (j = i-1; j >= 0; j--)
			{
				tem_3 = W[j][i];
				for (k = i; k < 2 * N; k++)
				{
					W[j][k] = W[j][k] - tem_3 * W[i][k];
				}
			}
		}
		
		for (i = 0; i < N; i++)
		{
			memcpy(&result.data[i][0], &W[i][N], N * sizeof(T));

		}
		
		for (i = 0; i < N; i++)
		{
			tem_1 *= W[i][i];
		}
		
		/// 矩阵不可逆，返回单位阵
		if(un_inv_flag)
		{
			result.zero();
		    result.ones();
		}
		
		return  result;
	}
	
	/**
	 * 当前M*N大小矩阵采用高斯消元法求逆，不可逆时返回单位阵
	 */
	Matrix<T, N, 	M> inverse() 
	{
        if (N != M)
           return *this;
        
		return Gaussian_elimination(data);
	}
	
	
	T data[M][N];
	uint8_t row;
	uint8_t col;
};

#endif