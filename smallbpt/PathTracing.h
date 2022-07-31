//#ifndef PATHTRACING_H
//#define PATHTRACING_H
//#include "Integrator.h"
//#include "Camera.h"
//#include "Sampler.h"
//
//class PathTracing : public Integrator {
//public:
//	PathTracing() {}
//	PathTracing(Sampler *smp, int maxdepth = 2, int _spp = 1) :
//		sampler(smp), maxDepth(maxdepth), spp(_spp) {}
//	void Render(const Scene &scene, const Camera &camera);
//	Vec3 radiance(const Scene& scene, const Camera& camera, const Ray &r, int depth, int E = 1);
//	Vec3 Li(const Scene& scene, const Camera& camera, const Ray &r, Sampler &sampler);
//private:
//	Sampler *sampler;
//	int spp;
//	int maxDepth;
//
//	bool visual = false;
//};
//
//Vec3 DirectIllumination(const Scene& scene, const Intersection &isect, const Vec3 &Throughput, Sampler& sampler);
//
//#endif
