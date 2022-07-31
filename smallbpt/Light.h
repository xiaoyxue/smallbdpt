#pragma once
#include "Geometry.h"

class Light {
public:

	virtual Vec3 Sample(Intersection* lightPoint, double *pdf, const Vec2& u) const = 0;

	virtual void SampleOnLight(Intersection* isect, Vec3* dir, double* pdfPos, double* pdfDir, const Vec2& u, const Vec2& v) const = 0;

	virtual Vec3 Emission() const = 0;
};

class AreaLight : public Light {
public:
	AreaLight(std::shared_ptr<Shape> pShape, const Vec3 &Le_) : mpShape(pShape), Le(Le_) {}

	Vec3 Sample(Intersection* lightPoint, double* pdf, const Vec2& u) const override;

	void SampleOnLight(Intersection* lightPoint, Vec3* dir, double* pdfPos, double* pdfDir, const Vec2& u, const Vec2& v) const override;

	Vec3 Emission() const override {
		return Le;
	}

private:
	std::shared_ptr<Shape> mpShape;
	Vec3 Le;
};
