#ifndef SAMPLER_H
#define SAMPLER_H
#include "Rng.h"
#include "Geometry.h"

#include "pbrtrng.h"

class Sampler {
public:
	Sampler(int seed = 1234) : rng(seed){}
	double Get1D();
	Vec3 Get2D();
private:
	Rng rng;
	RNG pbrtrng;
};

#endif
