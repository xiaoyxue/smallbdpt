#pragma once

#include "Intersection.h"

class Scene;
class VisibilityTester {
private:
	Intersection mP0, mP1;

public:
	VisibilityTester() {}

	VisibilityTester(const Intersection& p0, const Intersection& p1)
		: mP0(p0), mP1(p1)
	{
	}

	inline const Intersection& P0() const { return mP0; }

	inline const Intersection& P1() const { return mP1; }

	bool Unoccluded(const Scene& scene) const;

};