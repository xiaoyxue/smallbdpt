#ifndef INTERSECT_H
#define INTERSECT_H

#include "Geometry.h"
#include "BSDF.h"
#include "Smallbpt.h"

struct Intersect {
#ifdef _DEBUG
	int id;
#endif
	Vec3 HitPoint;
	Vec3 Normal;
	Vec3 SurfaceNormal;
	Vec3 wo;
	bool Delta;
	bool IsLight;
	std::shared_ptr<BSDF> bsdf;
	Intersect() {
		IsLight = false;
		Delta = false;
	}
};

#endif
