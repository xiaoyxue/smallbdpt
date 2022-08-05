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
		return (point - p).Norm();
	}

	bool Intersect(const Ray& r, Intersection* isect, double* t) const override;

	bool Intersect(const Ray& r) const override;

	double IntersectP(const Ray& r) const override;
private:
	double intersect(const Ray& r) const;
};


class Triangle : public Shape {
public:
	Triangle(Vec3 p0_, Vec3 p1_, Vec3 p2_, Vec3 normal_, Vec3 color_, Vec3 emission_, Refl_t refl_)
		: p0(p0_), p1(p1_), p2(p2_), normal(normal_), color(color_), emission(emission_), refl(refl_)
	{
		Vec3 e1 = p1 - p0;
		Vec3 e2 = p2 - p0;
	}

	bool Intersect(const Ray& ray, Intersection* isect, double* t) const override;


	bool Intersect(const Ray& ray) const override;

	double IntersectP(const Ray& ray) const override;

	double Area() const override {
		return e1.Cross(e2).Length() * 0.5;
	}

	Intersection Sample(double* pdf, const Vec3& u) const override;

	Vec3 Color() const override {
		return color;
	}

	Vec3 Emission() const override {
		return emission;
	}

	Vec3 GetNormal(const Vec3& point) const {
		return normal;
	}

	Refl_t ReflectType() const override {
		return refl;
	}

	Vec3 p0, p1, p2;
	Vec3 e1, e2;
	Vec3 normal;
	Vec3 color;
	Vec3 emission;
	Refl_t refl;
};


#endif