#pragma once

#pragma once
#include <cmath>
#include <algorithm>
#include <string>
#include <iostream>


struct Vec2 {
	double x, y;

	Vec2(double xx = 0.f, double yy = 0.f) : x(xx), y(yy) {}

	inline Vec2& operator=(const Vec2& b) {
		x = b.x; y = b.y;
		return *this;
	}
	inline Vec2 operator+(const Vec2& b) const {
		return Vec2(x + b.x, y + b.y);
	}
	inline Vec2 operator-(const Vec2& b) const {
		return Vec2(x - b.x, y - b.y);
	}
	inline Vec2 operator*(double c) const {
		return Vec2(c * x, c * y);
	}
	inline Vec2 operator/(double c) const {
		if (std::abs(c - 0.f) < Eps) return Vec2(0.f, 0.f);
		return Vec2(x / c, y / c);
	}
	inline Vec2& operator+=(const Vec2& b) {
		x += b.x;
		y += b.y;
		return *this;
	}
	inline Vec2& operator-=(const Vec2& b) {
		x -= b.x;
		y -= b.y;
		return *this;
	}

	inline bool operator==(const Vec2& b) const {
		if (x == b.x && y == b.y) return true;
		return false;
	}

	inline bool operator!=(const Vec2& b) const {
		if (x != b.x) return true;
		if (y != b.y) return true;
		return false;
	}

	inline double Dot(const Vec2& b) const {
		return x * b.x + y * b.y;
	}

	inline double operator[](int i) const {
		if (i == 0) return x;
		else return y;

	}
	inline double& operator[](int i) {
		if (i == 0) return x;
		else return y;
	}
};

inline Vec2 operator*(const Vec2& a, const Vec2& b) {
	return Vec2(a.x * b.x, a.y * b.y);
}

struct Vec3 {
	double x, y, z;

	Vec3(double xx = 0.f, double yy = 0.f, double zz = 0.f) : x(xx), y(yy), z(zz) {}

	Vec3(const Vec3& b) : x(b.x), y(b.y), z(b.z) {}

	inline Vec3& operator=(const Vec3& b) {
		x = b.x; y = b.y; z = b.z;
		return *this;
	}
	inline Vec3 operator+(const Vec3& b) const {
		return Vec3(x + b.x, y + b.y, z + b.z);
	}
	inline Vec3 operator-(const Vec3& b) const {
		return Vec3(x - b.x, y - b.y, z - b.z);
	}
	inline Vec3 operator*(double c) const {
		return Vec3(c * x, c * y, c * z);
	}
	inline Vec3 operator/(double c) const {
		if (std::abs(c - 0.f) < Eps) return Vec3(0.f, 0.f, 0.f);
		return Vec3(x / c, y / c, z / c);
	}
	inline Vec3& operator+=(const Vec3& b) {
		x += b.x;
		y += b.y;
		z += b.z;
		return *this;
	}
	inline Vec3& operator-=(const Vec3& b) {
		x -= b.x;
		y -= b.y;
		z -= b.z;
		return *this;
	}

	inline bool operator==(const Vec3& b) const {
		if (x == b.x && y == b.y && z == b.z) return true;
		return false;
	}

	inline bool operator!=(const Vec3& b) const {
		if (x != b.x) return true;
		if (y != b.y) return true;
		if (z != b.z) return true;
		return false;
	}

	inline double Dot(const Vec3& b) const {
		return x * b.x + y * b.y + z * b.z;
	}

	inline Vec3 operator%(const Vec3& b) const {
		return Vec3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
	}

	inline Vec3 Cross(const Vec3& b) const {
		return Vec3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
	}

	inline double operator[](int i) const {
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}
	inline double& operator[](int i) {
		if (i == 0) return x;
		if (i == 1) return y;
		return z;
	}
	inline Vec3 mult(const Vec3& b) const {
		return Vec3(x * b.x, y * b.y, z * b.z);
	}
	double Length() const {
		return std::sqrt(x * x + y * y + z * z);
	}
	inline Vec3 Norm() {
		return *this = (*this) / Length();
	}

	inline void Normalize() {
		*this = *this / this->Length();
	}

	inline double maxComponentValue() const {
		return std::max(x, std::max(y, z));
	}

	double Y() const {
		const real YWeight[3] = { (real)0.212671, (real)0.715160, (real)0.072169f };
		return YWeight[0] * this->x + YWeight[1] * this->y + YWeight[2] * this->z;
	}

	std::string ToString() const {
		std::string ret = "(";
		ret += std::to_string(x); ret += ", ";
		ret += std::to_string(y); ret += ", ";
		ret += std::to_string(z); ret += ")";
		return ret;
	}

	friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
		return os << v.ToString();
	}
};

inline double Dot(const Vec3& a, const Vec3& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 Cross(const Vec3& a, const Vec3& b) {
	return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline Vec3 operator*(const Vec3& a, const Vec3& b) {
	return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline Vec3 operator*(double a, const Vec3& b) {
	return Vec3(a * b.x, a * b.y, a * b.z);
}

inline void Normalize(Vec3& a) {
	a = a / a.Length();
}
