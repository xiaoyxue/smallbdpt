#ifndef INTEGRATOR_H
#define INTEGRATOR_H

class Scene;
class Camera;

class Integrator {
public:
	virtual ~Integrator() {};
	virtual void Render(const Scene &scene, const Camera &camera) = 0;
};


#endif