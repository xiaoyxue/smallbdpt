#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <cmath>
#include "Smallbpt.h"
#include <algorithm>
#include <string>
#include <iostream>
#include "Linagl.h"


class Ray {
public:
	Ray() : o(0.f, 0.f, 0.f), d(0.f, 0.f, 0.f) {}
	Ray(Vec3 _o, Vec3 _d, double _tmin = Eps, double _tmax = Inf) : 
		o(_o), d(_d), tmin(_tmin), tmax(_tmax) {}

	Vec3 HitPoint(double t) const {
		return o + d * t;
	}
public:
	Vec3 o, d;
	double tmin, tmax;
};

enum Refl_t { DIFF, SPEC, REFR };  // material types, used in radiance()

class Intersection;

class Shape {
public:
	virtual double Area() const = 0;
	virtual Intersection Sample(double* pdf, const Vec3& u) const = 0;
	virtual Vec3 Emission() const = 0;
	virtual Refl_t ReflectType() const = 0;
	virtual Vec3 Color() const = 0;
	virtual Vec3 GetNormal(const Vec3& point) const = 0;

	virtual bool Intersect(const Ray& r, Intersection* isect, double* t) const = 0;
	virtual bool Intersect(const Ray& r) const = 0;
	virtual double IntersectP(const Ray& r) const = 0;


};

class Sphere : public Shape{
public:
	double rad;       // radius

	Vec3 p, e, c;      // position, emission, color

	Refl_t refl;      // reflection type (DIFFuse, SPECular, REFRactive)

	Sphere(double rad_, Vec3 p_, Vec3 e_, Vec3 c_, Refl_t refl_) :
		rad(rad_), p(p_), e(e_), c(c_), refl(refl_) {}

	Intersection Sample(double* pdf, const Vec3& u) const override;

	double Area() const override {
		return 4.0 * PI * rad * rad;
	}

	Vec3 Emission() const override {
		return e;
	}

	Refl_t ReflectType() const override {
		return refl;
	}

	Vec3 Color() const override {
		return c;
	}

	Vec3 GetNormal(const Vec3& point) const override {
		return (point - p).norm();
	}

	bool Intersect(const Ray& r, Intersection* isect, double* t) const override;

	bool Intersect(const Ray& r) const override;

	double IntersectP(const Ray& r) const override;

	//double intersect(const Ray& r) const { // returns distance, 0 if no hit
	//	Vec3 op = p - r.o; // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
	//	double t, eps = 1e-4, b = op.dot(r.d), det = b * b - op.dot(op) + rad * rad;
	//	if (det < 0) return 0; else det = sqrt(det);
	//	return (t = b - det) > eps ? t : ((t = b + det) > eps ? t : 0);
	//}

};


//class Triangle : public Shape {
//public:
//	Triangle(Vec3 p0_, Vec3 p1_, Vec3 p2_, Vec3 normal_, Vec3 color_, Vec3 emission_, Refl_t refl_)
//		: p0(p0_), p1(p1_), p2(p2_), normal(normal_), color(color_), emission(emission_), refl(refl_)
//	{
//		Vec3 e1 = p1 - p0;
//		Vec3 e2 = p2 - p0;
//	}
//
//	bool intersect(const Ray& ray, Intersection* isect, double* t);
//
//
//	bool intersect(const Ray& ray) const {
//		Vec3 s1 = cross(ray.d, e2);
//		double divisor = dot(s1, e1);
//
//		//std::cout << "divisor: " << divisor << std::endl;
//
//		if (divisor == 0.)
//			return false;
//		double invDivisor = 1.f / divisor;
//
//		// Compute first barycentric coordinate
//		Vec3 s = ray.o - p0;
//		double b1 = dot(s, s1) * invDivisor;
//
//		//std::cout << "b1: " << b1 << std::endl;
//
//		if (b1 < 0. || b1 > 1.)
//			return false;
//
//		// Compute second barycentric coordinate
//		Vec3 s2 = cross(s, e1);
//		double b2 = dot(ray.d, s2) * invDivisor;
//		if (b2 < 0. || b1 + b2 > 1.)
//			return false;
//
//		// Compute _t_ to intersection point
//		double tHit = dot(e2, s2) * invDivisor;
//		if (tHit < ray.tmin || tHit > ray.tmax)
//			return false;
//		return true;
//	}
//
//	double Area() const override {
//		return e1.cross(e2).length() * 0.5;
//	}
//
//	Intersection Sample(double* pdf, const Vec3& u) const override;
//
//	Vec3 p0, p1, p2;
//	Vec3 e1, e2;
//	Vec3 normal;
//	Vec3 color;
//	Vec3 emission;
//	Refl_t refl;
//};


#endif