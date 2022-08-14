#pragma once

#include "Geometry.h"
#include "BSDF.h"
#include "Smallbpt.h"

class BSDF;
class Light;
class Intersection {
public:

	Intersection() {}

	Intersection(const Vec3 o, double time) : mPos(o), mTime(time) {}

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
	bool mIsLight;
	double b1, b2;
	double mTime;
	std::shared_ptr<Light> pLight;
	std::shared_ptr<BSDF> mpBSDF;
};

class SurfaceIntersection : public Intersection {
public:
	Shape* shape;
};

