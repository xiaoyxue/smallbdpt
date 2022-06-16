#ifndef SCENE_H
#define SCENE_H

#include "Geometry.h"
#include "Intersect.h"

extern Sphere spheres[];

extern int numSpheres;

bool intersect(const Ray &r, double &t, int &id, Intersect &isect); 

#ifdef _DEBUG
bool intersect(const Ray &r, int &_id);
#else
bool intersect(const Ray &r);
#endif
//inline double clamp(double x) { return x<0 ? 0 : x>1 ? 1 : x; }

//inline int toInt(double x) { return int(pow(clamp(x), 1 / 2.2) * 255 + .5); }

#endif