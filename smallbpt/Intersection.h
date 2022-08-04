#ifndef INTERSECT_H
#define INTERSECT_H

#include "Geometry.h"
#include "BSDF.h"
#include "Smallbpt.h"

class BSDF;
class Intersection {
public:
#ifdef _DEBUG
	int id;
#endif
	Vec3 HitPoint;
	Vec3 Normal;
	Vec3 SurfaceNormal;
	Vec3 wo;
	bool Delta;
	bool IsLight;
	double b1, b2;
	std::shared_ptr<BSDF> bsdf;
	Intersection() {
		IsLight = false;
		Delta = false;
	}
};

#endif
