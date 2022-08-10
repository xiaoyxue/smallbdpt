#ifndef PATHVERTEX_H
#define PATHVERTEX_H
#include "Intersection.h"
#include "Geometry.h"
#include "Camera.h"



class Light;
class Camera;

class PathVertex {
public:
	Intersection mIsect;
	Vec3 mThroughput;
	double mPdfForward = 0.0, mPdfBackward = 0.0;
	Camera* mpCamera;
	Light* mpLight;
	bool mIsDelta;
	double G;
};


class BDPTMISNode {
public:
	BDPTMISNode(float towardLight = 0.0f, float towardEye = 0.0f, bool spec = false)
		: pTowardLight(towardLight), pTowardEye(towardEye), isSpecular(spec) {}
	double pTowardLight;
	double pTowardEye;
	bool isSpecular;
};

#endif