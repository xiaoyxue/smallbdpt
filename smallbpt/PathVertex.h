#ifndef PATHVERTEX_H
#define PATHVERTEX_H
#include "Intersection.h"
#include "Geometry.h"
#include "Camera.h"

struct PathVertex {
	Intersection isect;
	Vec3 mThroughput;
	double mPdfFwd = 0.0, mPdfPrev = 0.0;
	Camera *mpCamera;
};


#endif