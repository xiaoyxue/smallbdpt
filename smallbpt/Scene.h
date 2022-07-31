#ifndef SCENE_H
#define SCENE_H

#include "Geometry.h"
#include "Intersection.h"
#include <vector>

class Light;
class Scene {
public:
	static std::vector<std::shared_ptr<Triangle>> triangles;
	static std::vector<std::shared_ptr<Light>> lights;
	bool intersect(const Ray& r, double& t, Intersection& isect) const;
	bool intersect(const Ray& r) const;
	std::shared_ptr<Light> SampleOneLight(double *pdfLight, double u) const;
};



#endif