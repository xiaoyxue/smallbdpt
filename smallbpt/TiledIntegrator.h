#pragma once
#include "Integrator.h"
#include "Sampler.h"

struct Tile {
	int minX, minY, maxX, maxY;
	Tile() = default;
	Tile(int minX, int minY, int maxX, int maxY) : minX(minX), minY(minY), maxX(maxX), maxY(maxY) {}
};


class TiledIntegrator : public Integrator
{
protected:
	int mSpp;
	Sampler* mpSampler;

public:
	TiledIntegrator() = default;
	TiledIntegrator(int spp, Sampler *pSampler) :
		mSpp(spp), mpSampler(pSampler)
	{
	}
	virtual ~TiledIntegrator() {}
	virtual void Render(const Scene& scene, const Camera& camera) override;
protected:
	virtual Vec3 Li(const Ray& ray, const Scene& scene, Sampler &sampler) const = 0;
};
