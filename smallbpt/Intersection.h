#ifndef INTERSECT_H
#define INTERSECT_H

#include "Geometry.h"
#include "BSDF.h"
#include "Smallbpt.h"

class BSDF;
class Light;
class Intersection {
public:
	Intersection() {
		IsLight = false;
		mIsDelta = false;
	}

	Ray SpawnRay(const Vec3& d) const {
		return Ray(mPos + d * RayEps, d);
	}

	Ray SpawnTo(const Intersection& it) const {
		Vec3 dir = (it.mPos - mPos);
		double dis = dir.Length();
		return Ray(mPos + RayEps * mNormal + RayEps * dir.Norm(), dir.Norm(), RayEps, dis);
	}

	Vec3 mPos;
	Vec3 mNormal;
	Vec3 mSurfaceNormal;
	Vec3 mOutDir;
	bool mIsDelta;
	bool IsLight;
	Shape* pLight;
	double b1, b2;
	std::shared_ptr<BSDF> mpBSDF;

};

#endif
