#pragma once

#include "Geometry.h"
#include "Intersection.h"
#include "Sampler.h"

class PathVertex;
class Scene;
class Light {
public:
	virtual Vec3 Sample(Intersection* lightPoint, double* pdf, const Vec3& u) const = 0;
	virtual Vec3 SampleFromLight(Intersection* lightPoint, Vec3* dir, double* pdfPos, double* pdfDir, double *cosTheta, Sampler& sampler) const = 0;
	virtual Vec3 Emission() const = 0;
	virtual Shape* GetShape() const = 0;
	virtual Vec3 DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3 &throughput, PathVertex *sampled) const = 0;
};

class SphereLight : public Light {
public:
	SphereLight(Sphere* pSphere) : mpSphere(pSphere) {}

	Vec3 Sample(Intersection* lightPoint, double* pdf, const Vec3& u) const override;

	Vec3 Emission() const override {
		return mpSphere->Emission();
	}

	Shape* GetShape() const  override {
		return mpSphere;
	}

	Vec3 SampleFromLight(Intersection* lightPoint, Vec3* dir, double* pdfPos, double* pdfDir, double *cosTheta, Sampler& sampler) const override;

	Vec3 DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3 &throughput, PathVertex *sampled) const override;

private:
	Sphere* mpSphere;
};





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
