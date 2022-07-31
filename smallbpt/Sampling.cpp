#include "Sampling.h"
#include "Smallbpt.h"
#include <algorithm>

Vec3 UniformSampleDisk(const Vec3 &u) {
	double r = std::sqrt(u[0]);
	double theta = 2 * PI * u[1];
	return Vec3(r * std::cos(theta), r * std::sin(theta));
}

Vec2 ConcentricSampleDisk(const Vec2 &u) {
	// Map uniform random numbers to $[-1,1]^2$
	Vec2 uOffset = 2.0 * u - Vec2(1, 1);

	// Handle degeneracy at the origin
	if (uOffset.x == 0 && uOffset.y == 0) return Vec2(0, 0);

	// Apply concentric mapping to point
	double theta, r;
	if (std::abs(uOffset.x) > std::abs(uOffset.y)) {
		r = uOffset.x;
		theta = PiOver4 * (uOffset.y / uOffset.x);
	}
	else {
		r = uOffset.y;
		theta = PiOver2 - PiOver4 * (uOffset.x / uOffset.y);
	}
	return r * Vec2(std::cos(theta), std::sin(theta));
}

Vec3 UniformSampleHemisphere(const Vec3 &u) {
	double z = u[0];
	double r = std::sqrt(std::max((double)0, (double)1. - z * z));
	double phi = 2 * PI * u[1];
	return Vec3(r * std::cos(phi), r * std::sin(phi), z);
}

double UniformHemispherePdf() { return INV2PI; }

Vec3 UniformSampleSphere(const Vec3 &u) {
	double z = 1 - 2 * u[0];
	double r = std::sqrt(std::max((double)0, (double)1 - z * z));
	double phi = 2 * PI * u[1];
	return Vec3(r * std::cos(phi), r * std::sin(phi), z);
}

double UniformSpherePdf() { return INV4PI; }

Vec3 UniformSampleCone(const Vec3 &u, double cosThetaMax) {
	double cosTheta = ((double)1 - u[0]) + u[0] * cosThetaMax;
	double sinTheta = std::sqrt((double)1 - cosTheta * cosTheta);
	double phi = u[1] * 2 * PI;
	return Vec3(std::cos(phi) * sinTheta, std::sin(phi) * sinTheta,
		cosTheta);
}
Vec3 UniformSampleCone(const Vec3 &u, double cosThetaMax, const Vec3 &x,
	const Vec3 &y, const Vec3 &z) {
	double cosTheta = Lerp(u[0], cosThetaMax, 1.f);
	double sinTheta = std::sqrt((double)1. - cosTheta * cosTheta);
	double phi = u[1] * 2 * PI;
	return std::cos(phi) * sinTheta * x + std::sin(phi) * sinTheta * y +
		cosTheta * z;
}
double UniformConePdf(double cosThetaMax) {
	return 1 / (2 * PI * (1 - cosThetaMax));
}

Vec2 UniformSampleTriangle(const Vec2& u) {
	double su0 = std::sqrt(u.x);
	return Vec2(1 - su0, u.y * su0);
}
