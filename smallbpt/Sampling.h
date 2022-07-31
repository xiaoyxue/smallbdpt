#ifndef SAMPLING_H
#define SAMPLING_H
#include "Geometry.h"
#include <algorithm>

Vec3 UniformSampleDisk(const Vec3 &u);
Vec3 ConcentricSampleDisk(const Vec3 &u);

Vec3 UniformSampleHemisphere(const Vec3 &u);
double UniformHemispherePdf();

Vec3 UniformSampleSphere(const Vec3 &u);
double UniformSpherePdf();


Vec3 UniformSampleCone(const Vec3 &u, double cosThetaMax);
Vec3 UniformSampleCone(const Vec3 &u, double cosThetaMax, const Vec3 &x, const Vec3 &y, const Vec3 &z);
double UniformConePdf(double cosThetaMax);

inline Vec3 CosineSampleHemisphere(const Vec3 &u) {
	Vec3 d = ConcentricSampleDisk(u);
	double z = std::sqrt(std::max((double)0, 1 - d.x * d.x - d.y * d.y));
	//std::cout << z << std::endl;
	return Vec3(d.x, d.y, z);
}

inline double CosineHemispherePdf(double cosTheta) { return cosTheta * INV_PI; }

Vec3 UniformSampleTriangle(const Vec3& u);

#endif