#pragma once

#include "TiledIntegrator.h"


class PathTracing : public TiledIntegrator {
private:
	int mMaxDepth;
public:
	PathTracing(int spp, int maxDepth, Sampler *pSampler) : TiledIntegrator(spp, pSampler), mMaxDepth(maxDepth) {}

	Vec3 Li(const Scene& scene, const Ray& r) const override;
};