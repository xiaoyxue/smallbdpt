#pragma once
#include "Geometry.h"
#include "Intersection.h"

//class Light {
//public:
//	virtual Vec3 Sample(Intersection* lightPoint, double *pdf, const Vec2& u) const = 0;
//};
//
//class AreaLight : public Light {
//public:
//	AreaLight(Shape* pShape) : mpShape(pShape) {}
//
//	Vec3 Sample(Intersection* lightPoint, double* pdf, const Vec2& u) const override {
//		*lightPoint = mpShape->Sample(pdf, u);
//	}
//
//private:
//	Shape* mpShape;
//};
