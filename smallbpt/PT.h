#pragma once

#include "TiledIntegrator.h"


class PathTracing : public TiledIntegrator {
private:
	int mMaxDepth;
public:
	PathTracing(Sampler* pSampler, int spp, int maxDepth) : TiledIntegrator(spp, pSampler), mMaxDepth(maxDepth) {}

	Vec3 Li(const Ray& r, const Scene& scene, Sampler& sampler) const override;
};