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
	virtual Vec3 DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3 &throughput, PathVertex *sampled = 0) const = 0;
	virtual void PdfLe(const Ray& ray, const Vec3& n, double* pdfPos, double* pdfDir) const = 0;
};


class AreaLight : public Light {
public:
	AreaLight(Shape* pShape) : mpShape(pShape) {}

	Vec3 Sample(Intersection* lightPoint, double* pdf, const Vec3& u) const override;

	Vec3 Emission() const override {
		return mpShape->Emission();
	}

	Shape* GetShape() const  override {
		return mpShape;
	}

	Vec3 SampleFromLight(Intersection* lightPoint, Vec3* dir, double* pdfPos, double* pdfDir, double* cosTheta, Sampler& sampler) const override;

	Vec3 DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3& throughput, PathVertex* sampled = 0) const override;

	void PdfLe(const Ray& ray, const Vec3& n, double* pdfPos, double* pdfDir) const override;
private:
	Shape* mpShape;
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

	Vec3 DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3 &throughput, PathVertex *sampled = 0) const override;

	void PdfLe(const Ray& ray, const Vec3& n, double* pdfPos, double* pdfDir) const override {}

private:
	Sphere* mpSphere;
};
