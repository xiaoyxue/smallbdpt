#include "Sampler.h"

double Sampler::Get1D() {
	return rng.Getdouble();
}

Vec2 Sampler::Get2D()
{
	return Vec2(Get1D(), Get1D());
}

Vec3 Sampler::Get3D()
{
	return Vec3(Get1D(), Get1D(), Get1D());
}
