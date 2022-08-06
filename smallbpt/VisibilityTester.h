#pragma once

#include "Intersection.h"
#include "Scene.h"

class VisibilityTester {
private:
	Intersection mP0, mP1;
public:
	VisibilityTester(const Intersection& p0, const Intersection& p1)
		: mP0(p0), mP1(p1)
	{
	}
	const Intersection& P0() const { return mP0; }

	const Intersection& P1() const { return mP1; }

	bool Unoccluded(const Scene& scene) const {
		Ray ray = mP0.SpawnTo(mP1);
		return !scene.Intersect(ray);
	}

};