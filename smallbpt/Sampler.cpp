#include "Sampler.h"

double Sampler::Get1D() {
	return rng.Getdouble();
	//return pbrtrng.UniformFloat();
}

Vec3 Sampler::Get2D() {
	return Vec3(Get1D(), Get1D(), 0.f);
}