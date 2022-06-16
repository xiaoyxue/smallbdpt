#ifndef SMALLBPT_H
#define SMALLBPT_H

#include <memory>
#include <limits>

const double eps = 10e-6;
const double PI = 3.14159265358979323846;
const double INV_PI = 0.31830988618379067154;
const double INV2PI = 0.15915494309189533577;
const double INV4PI = 0.07957747154594766788;
const double PiOver2 = 1.57079632679489661923;
const double PiOver4 = 0.78539816339744830961;
const double Sqrt2 = 1.41421356237309504880;
const double Inf = 1e20;

inline double clamp(double x) { return x<0 ? 0 : x>1 ? 1 : x; }

//inline int toInt(double x) { return int(pow(clamp(x), 1 / 2.2) * 255 + .5); }

// tone mapping and gamma correction
inline int toInt(double x) {
	return int(pow(1 - exp(-x), 1 / 2.2) * 255 + .5);
}

inline double Lerp(double t, double v1, double v2) { return (1 - t) * v1 + t * v2; }


template <typename T, typename U, typename V>
inline T Clamp(T val, U low, V high) {
	if (val < low)
		return low;
	else if (val > high)
		return high;
	else
		return val;
}


#endif