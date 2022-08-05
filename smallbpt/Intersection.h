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
	Vec3 mPos;
	Vec3 mNormal;
	Vec3 mSurfaceNormal;
	Vec3 wo;
	bool mIsDelta;
	bool IsLight;
	Shape* pLight;
	double b1, b2;
	std::shared_ptr<BSDF> bsdf;
	Intersection() {
		IsLight = false;
		mIsDelta = false;
	}
};

#endif
