#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <cmath>
#include "Smallbpt.h"
#include <algorithm>
#include <string>

struct Vec3 {        // Usage: time ./explicit 16 && xv image.ppm
	double x, y, z;
	
	Vec3(double xx = 0.f, double yy = 0.f, double zz = 0.f) : x(xx), y(yy), z(zz) {}
	Vec3(const Vec3 &b) : x(b.x), y(b.y), z(b.z) {}
	inline Vec3& operator=(const Vec3 &b) {
		x = b.x; y = b.y; z = b.z;
		return *this;
	}
	inline Vec3 operator+(const Vec3 &b) const {
		return Vec3(x + b.x, y + b.y, z + b.z);
	}
	inline Vec3 operator-(const Vec3 &b) const {
		return Vec3(x - b.x, y - b.y, z - b.z);
	}
	inline Vec3 operator*(double c) const {
		return Vec3(c * x, c * y, c * z);
	}
	inline Vec3 operator/(double c) const {
		if (std::abs(c - 0.f) < eps) return Vec3(0.f, 0.f, 0.f);
		return Vec3(x / c, y / c, z / c);
	}
	inline Vec3& operator+=(const Vec3 &b) {
		x += b.x;
		y += b.y;
		z += b.z;
		return *this;
	}
	inline Vec3& operator-=(const Vec3 &b) {
		x -= b.x;
		y -= b.y;
		z -= b.z;
		return *this;
	}

	inline bool operator==(const Vec3 &b) const {
		if (x == b.x && y == b.y && z == b.z) return true;
		return false;
	}

	inline bool operator!=(const Vec3 &b) const {
		if (x != b.x) return true;
		if (y != b.y) return true;
		if (z != b.z) return true;
		return false;
	}

	inline double dot(const Vec3 &b) const {
		return x * b.x + y * b.y + z * b.z;
	}
	inline Vec3 operator%(const Vec3 &b) const {
		return Vec3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
	}
	inline double operator[](int i) const {
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}
	inline double &operator[](int i) {
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}
	inline Vec3 mult(const Vec3 &b) const {
		return Vec3(x * b.x, y * b.y, z * b.z);
	}
	double length() const {
		return std::sqrt(x * x + y * y + z * z);
	}
	inline Vec3 norm() {
		return *this = (*this) / length();
	}
	inline double maxComponentValue() const {
		return std::max(x, std::max(y, z));
	}
	std::string ToString() const {
		std::string ret = "(";
		ret += std::to_string(x); ret += ", ";
		ret += std::to_string(y); ret += ", ";
		ret += std::to_string(z); ret += ")";
		return ret;
	}
	friend std::ostream &operator<<(std::ostream &os, const Vec3 &v) {
		return os << v.ToString();
	}
};

typedef Vec3 Color;

Vec3 operator*(const Vec3 &a, const Vec3 &b);
Vec3 operator*(double a, const Vec3 &);
void Normalize(Vec3 &Vec3);


class Ray {
public:
	Ray() : o(0.f, 0.f, 0.f), d(0.f, 0.f, 0.f) {}
	Ray(Vec3 _o, Vec3 _d, double _tmin = eps, double _tmax = Inf) : 
		o(_o), d(_d), tmin(_tmin), tmax(_tmax) {}

	Vec3 HitPoint(double t) const {
		return o + d * t;
	}
public:
	Vec3 o, d;
	double tmin, tmax;
};

enum Refl_t { DIFF, SPEC, REFR };  // material types, used in radiance()

struct Sphere {
	double rad;       // radius
	Vec3 p, e, c;      // position, emission, color
	Refl_t refl;      // reflection type (DIFFuse, SPECular, REFRactive)
	Sphere(double rad_, Vec3 p_, Vec3 e_, Vec3 c_, Refl_t refl_) :
		rad(rad_), p(p_), e(e_), c(c_), refl(refl_) {}
	double intersect(const Ray &r) const { // returns distance, 0 if nohit
		Vec3 op = p - r.o; // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
		double t, eps = 1e-4, b = op.dot(r.d), det = b*b - op.dot(op) + rad*rad;
		if (det<0) return 0; else det = sqrt(det);
		return (t = b - det)>eps ? t : ((t = b + det)>eps ? t : 0);
	}
};

inline void CoordinateSystem(const Vec3 &v1, Vec3 *v2, Vec3 *v3) {
	if (std::abs(v1.x) > std::abs(v1.y))
		*v2 = Vec3(-v1.z, 0, v1.x) / std::sqrt(v1.x * v1.x + v1.z * v1.z);
	else
		*v2 = Vec3(0, v1.z, -v1.y) / std::sqrt(v1.y * v1.y + v1.z * v1.z);
	*v3 = v1 % (*v2);
}
#endif