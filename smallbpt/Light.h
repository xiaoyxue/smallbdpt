#pragma once
#include "Geometry.h"

class Light {
	virtual Vec3 Sample(Intersection* lightPoint, double *pdf, const Vec3& u) = 0;
};

class AreaLight {
public:
	AreaLight(std::shared_ptr<Shape> pShape, const Vec3 &Le_) : mpShape(pShape), Le(Le_) {}

	Vec3 Sample(Intersection* lightPoint, double* pdf, const Vec3& u) {
		*lightPoint = mpShape->Sample(pdf, u);
	}

private:
	std::shared_ptr<Shape> mpShape;
	Vec3 Le;
};
