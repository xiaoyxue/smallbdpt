#ifndef SMALLBPT_H
#define SMALLBPT_H

#include <memory>
#include <limits>


constexpr double PI = 3.14159265358979;
constexpr double INV_PI = 0.31830988618379067154;
constexpr double INV_2PI = 0.15915494309189533577;
constexpr double INV_4PI = 0.07957747154594766788;
constexpr double PI_Over2 = 1.57079632679489661923;
constexpr double PI_Over4 = 0.78539816339744830961;
constexpr double Eps = 1e-6;
constexpr double Inf = std::numeric_limits<double>::infinity();
constexpr double RayEps = 1e-4;
constexpr double ShadowRayEps = 1e-4;
constexpr double PhtotonEdgeEps = 0.0009;
constexpr double NumericalEps = 1e-6;
constexpr double MachineEps = std::numeric_limits<double>::epsilon() * 0.5;
constexpr double MaxReal = std::numeric_limits<double>::max();
constexpr double Infinity = std::numeric_limits<double>::infinity();
constexpr double eps = 1e-5;


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