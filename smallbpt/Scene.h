#ifndef SCENE_H
#define SCENE_H

#include "Geometry.h"
#include "Intersection.h"

class Light;
class Scene {
public:
	static int numSpheres;
	static Sphere spheres[];

	static std::vector<Shape*> shapes;
	static std::vector<Light*> lights;

	bool Intersect(const Ray& r, double& t, Intersection& isect) const;
	bool Intersect(const Ray& r) const;
	Light* SampleOneLight(double* pdfLight, double u) const;
};



#endif