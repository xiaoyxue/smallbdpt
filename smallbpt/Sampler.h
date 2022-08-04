#ifndef SAMPLER_H
#define SAMPLER_H
#include "Rng.h"
#include "Geometry.h"

class Sampler {
public:
	Sampler(int seed = 1234) : rng(seed){}
	double Get1D() {
		return rng.GetFloat();
	}

	Vec2 Get2D() {
		return Vec2(Get1D(), Get1D());
	}

	Vec3 Get3D() {
		return Vec3(Get1D(), Get1D(), Get1D());
	}

	Sampler* Clone(int seed) {
		return new Sampler(seed);
	}
private:
	Rng rng;
};

#endif
