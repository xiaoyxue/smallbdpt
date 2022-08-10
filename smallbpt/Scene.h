#ifndef SCENE_H
#define SCENE_H

#include "Geometry.h"
#include "Intersection.h"

class Light;
class Scene {
public:
	static int numSpheres;
	static Sphere spheres[];

	static std::vector<std::shared_ptr<Shape>> shapes;
	static std::vector<std::shared_ptr<Light>> lights;

	bool Intersect(const Ray& r, Intersection* isect) const;
	bool Intersect(const Ray& r) const;
	std::shared_ptr<Light> SampleOneLight(double* pdfLight, double u) const;
};



#endif