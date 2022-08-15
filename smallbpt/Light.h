#pragma once

#include "Geometry.h"
#include "Intersection.h"
#include "Sampler.h"


class PathVertex;
class Scene;
class VisibilityTester;

class Light {
public:
	virtual Vec3 Sample(Intersection* lightPoint, double* pdf, const Vec3& u) const = 0;
	virtual Vec3 SampleFromLight(Intersection* lightPoint, Vec3* dir, double* pdfPos, double* pdfDir, double *cosTheta, Sampler& sampler) const = 0;
	virtual Vec3 Emission() const = 0;
	virtual Shape* GetShape() const = 0;
	virtual Vec3 DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3 &throughput, PathVertex *sampled = 0) const = 0;
	virtual void PdfLe(const Ray& ray, const Vec3& n, double* pdfPos, double* pdfDir) const = 0;
	virtual Vec3 L(const Intersection& it, const Vec3& w) const = 0;
	virtual Vec3 SampleLe(const Vec3& u1, const Vec3& u2, Ray* ray, Vec3* nLight, double* pdfPos, double* pdfDir) const = 0;
	virtual Vec3 SampleLi(const Intersection& ref, const Vec3& u, Vec3* wi, double* pdf, VisibilityTester *vis) const = 0;
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

	Vec3 SampleLe(const Vec3& u1, const Vec3& u2, Ray* ray, Vec3* nLight, double* pdfPos, double* pdfDir) const override;

	Vec3 SampleLi(const Intersection& ref, const Vec3& u, Vec3* wi, double* pdf, VisibilityTester* vis) const override;

	Vec3 L(const Intersection& it, const Vec3& w) const override { return Emission(); }
private:
	Shape* mpShape;
};



//class SphereLight : public Light {
//public:
//	SphereLight(Sphere* pSphere) : mpSphere(pSphere) {}
//
//	Vec3 Sample(Intersection* lightPoint, double* pdf, const Vec3& u) const override;
//
//	Vec3 Emission() const override {
//		return mpSphere->Emission();
//	}
//
//	Shape* GetShape() const  override {
//		return mpSphere;
//	}
//
//	Vec3 SampleFromLight(Intersection* lightPoint, Vec3* dir, double* pdfPos, double* pdfDir, double *cosTheta, Sampler& sampler) const override;
//
//	Vec3 DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3 &throughput, PathVertex *sampled = 0) const override;
//
//	void PdfLe(const Ray& ray, const Vec3& n, double* pdfPos, double* pdfDir) const override {}
//
//	Vec3 L(const Intersection& it, const Vec3& w) const override {
//		return Emission();
//	}
//
//	Vec3 SampleLi(const Intersection& ref, const Vec3& u, Vec3* wi, double* pdf, VisibilityTester* vis) const override;
//
//	Vec3 SampleLe(const Vec3& u1, const Vec3& u2, double time, Ray* ray, Vec3* nLight, double* pdfPos, double* pdfDir) const override;
//
//private:
//	Sphere* mpSphere;
//};
