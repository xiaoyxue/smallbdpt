#ifndef SCENE_H
#define SCENE_H

#include "Geometry.h"
#include "Intersection.h"

class Scene {
public:
	static int numSpheres;
	static Sphere spheres[];
	bool intersect(const Ray& r, double& t, int& id, Intersection& isect) const;
	bool intersect(const Ray& r) const;

};



#endif