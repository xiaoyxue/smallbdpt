#ifndef SAMPLER_H
#define SAMPLER_H
#include "Rng.h"
#include "Geometry.h"


class Sampler {
public:
	Sampler(int seed = 1234) : rng(seed){}

	double Get1D();

	Vec2 Get2D();

	Vec3 Get3D();

private:
	Rng rng;
};

#endif
