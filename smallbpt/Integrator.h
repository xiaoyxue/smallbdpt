#ifndef INTEGRATOR_H
#define INTEGRATOR_H

class Integrator {
public:
	virtual ~Integrator() {};
	virtual void Render() = 0;
};


#endif