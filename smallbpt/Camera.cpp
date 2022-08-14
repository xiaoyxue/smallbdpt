#include "Camera.h"
#include <cmath>
#include "Smallbpt.h"
#include <iostream>

void Camera::SetCamera(const Vec3 &_o, const Vec3 &_d,
	const Vec3 &_u, const Vec3 &_v, double _dis, double fov) {
	o = _o;
	d = _d;
	u = _u;
	v = _v;
	dis = _dis;
	fovy = fov;
}

void Camera::Init() {


	Vec3 ScreenCenter = o + d * dis;
	double filmHeigh = dis * tan(fovy / 2.0 * PI / 180) * 2.f;
	double filmWidth = filmHeigh * film->aspect;

	std::cout << filmHeigh / 2.f << std::endl;

	film->LU = ScreenCenter + v * filmHeigh / 2.0 - u * filmWidth / 2.0;
	film->LL = ScreenCenter - v * filmHeigh / 2.0 - u * filmWidth / 2.0;
	film->RU = ScreenCenter + v * filmHeigh / 2.0 + u * filmWidth / 2.0;
	film->RL = ScreenCenter - v * filmHeigh / 2.0 + u * filmWidth / 2.0;

	std::cout << "LU: " << film->LU.x << ", " << film->LU.y << ", " << film->LU.z << std::endl;
	std::cout << "LL: " << film->LL.x << ", " << film->LL.y << ", " << film->LL.z << std::endl;
	std::cout << "RU: " << film->RU.x << ", " << film->RU.y << ", " << film->RU.z << std::endl;
	std::cout << "RL: " << film->RL.x << ", " << film->RL.y << ", " << film->RL.z << std::endl;


	film->Area = (film->LL - film->LU).Length() * (film->RU - film->LU).Length();

	tanHalfFov = std::tan(fovy / 2);

	aspectRatio = film->aspect;
}

int Camera::GenerateCameraRay(const CameraSample &sample, Ray &ray) const {
	Vec3 pFilm = Vec3(sample.pFilm.x, sample.pFilm.y, 0);
	Vec3 dir = (pFilm - o).Norm();
	ray.o = o;
	ray.d = dir;
	return 1;
}