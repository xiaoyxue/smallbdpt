#ifndef PATHVERTEX_H
#define PATHVERTEX_H
#include "Intersection.h"
#include "Geometry.h"
#include "Camera.h"

class PathVertex {
public:
	Intersection mIsect;
	Vec3 mThroughput;
	double mPdfFwd = 0.0, mPdfPrev = 0.0;
	Camera *mpCamera;
	Light* mpLight;
};


#endif