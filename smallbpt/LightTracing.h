#ifndef LIGHTTRACING_H
#define LIGHTTRACING_H

#include "Integrator.h"
#include "Camera.h"
#include "Sampler.h"
#include "Intersect.h"
#include <vector>
#include "Geometry.h"
#include "PathVertex.h"

class LightTracing : public Integrator {
public:
	LightTracing(){}
	LightTracing(Sampler *smp, Camera *cam, int maxdepth, int samplePerPixel) : 
		sampler(smp), camera(cam), maxDepth(maxdepth), spp(samplePerPixel), LightPath(maxdepth + 1){}
	void Render();
	static int GenerateLightPath(Sampler &sampler, std::vector<PathVertex> &LightPath, int maxdepth);
	static int Trace(const Ray &ray, Vec3 Throughput, double PdfFwd, Sampler &sampler, std::vector<PathVertex> &LightPath, int depth, int maxDepth);
	static Vec3 ConnectToCamera(const PathVertex &Vertex, int s, const Camera &camera, Vec3 *pRaster, bool *inScreen);

private:
	int spp, maxDepth;
	Camera *camera;
	Sampler *sampler;
	std::vector<PathVertex> LightPath;
};


#endif