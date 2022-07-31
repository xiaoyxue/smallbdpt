#ifndef LIGHTTRACING_H
#define LIGHTTRACING_H

#include "Integrator.h"
#include "Camera.h"
#include "Sampler.h"
#include "Intersection.h"
#include <vector>
#include "Geometry.h"
#include "PathVertex.h"

class LightTracing : public Integrator {
public:
	LightTracing(){}
	LightTracing(Sampler *smp, int maxdepth, int samplePerPixel) : 
		sampler(smp), maxDepth(maxdepth), spp(samplePerPixel), LightPath(maxdepth + 1){}
	void Render(const Scene &scene, const Camera &camera)override;
	static int GenerateLightPath(const Scene& scene, Sampler &sampler, std::vector<PathVertex> &LightPath, int maxdepth);
	static int Trace(const Scene &scene, const Ray &ray, Vec3 Throughput, double PdfFwd, Sampler &sampler, std::vector<PathVertex> &LightPath, int depth, int maxDepth);
	static Vec3 ConnectToCamera(const Scene &scene, const PathVertex &Vertex, int s, const Camera &camera, Vec3 *pRaster, bool *inScreen);

private:
	int spp, maxDepth;
	Sampler *sampler;
	std::vector<PathVertex> LightPath;
};


#endif