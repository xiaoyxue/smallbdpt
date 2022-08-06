#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include "Scene.h"

class Camera;
class Integrator {
public:
	virtual ~Integrator() {};

	virtual void Render(const Scene &scene, const Camera &camera) = 0;


};


Vec3 SimpleDirectIllumination(const Scene& scene, const Intersection& hitPoint, Sampler& sampler);


#endif