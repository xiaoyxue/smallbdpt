#include "Geometry.h"

Vec3 operator*(const Vec3 &a, const Vec3 &b) {
	return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

Vec3 operator*(double a, const Vec3 &b) {
	return Vec3(a * b.x, a * b.y, a * b.z);
}

void Normalize(Vec3 &a) {
	a = a / a.length();
}