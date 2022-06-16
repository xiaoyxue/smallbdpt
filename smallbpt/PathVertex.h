#ifndef PATHVERTEX_H
#define PATHVERTEX_H
#include "Intersect.h"
#include "Geometry.h"
#include "Camera.h"

struct PathVertex {
	Intersect isect;
	Vec3 Throughput;
	double PdfFwd = 0.0, PdfPrev = 0.0;
	Camera *camera;
};


#endif