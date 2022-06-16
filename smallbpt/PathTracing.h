#ifndef PATHTRACING_H
#define PATHTRACING_H
#include "Integrator.h"
#include "Camera.h"
#include "Sampler.h"

class PathTracing : public Integrator {
public:
	PathTracing() {}
	PathTracing(Sampler *smp, Camera *cam, int maxdepth = 2, int _spp = 1) :
		sampler(smp), camera(cam), maxDepth(maxdepth), spp(_spp) {}
	void Render();
	Vec3 radiance(const Ray &r, int depth, int E = 1);
	Vec3 Li(const Ray &r, Sampler &sampler);
private:
	Camera *camera;
	Sampler *sampler;
	int spp;
	int maxDepth;

	bool visual = false;
};

Vec3 DirectIllumination(const Intersect &isect, const Vec3 &Throughput, Sampler& sampler);

#endif
