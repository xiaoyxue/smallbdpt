#include "Sampling.h"
#include "Smallbpt.h"
#include <algorithm>

Vec3 UniformSampleDisk(const Vec3& u) {
	double r = std::sqrt(u[0]);
	double theta = 2 * PI * u[1];
	return Vec3(r * std::cos(theta), r * std::sin(theta));
}

Vec3 ConcentricSampleDisk(const Vec3& u) {
	// Map uniform random numbers to $[-1,1]^2$
	Vec3 uOffset = 2.f * u - Vec3(1, 1);

	// Handle degeneracy at the origin
	if (uOffset.x == 0 && uOffset.y == 0) return Vec3(0, 0);

	// Apply concentric mapping to point
	double theta, r;
	if (std::abs(uOffset.x) > std::abs(uOffset.y)) {
		r = uOffset.x;
		theta = PI_Over4 * (uOffset.y / uOffset.x);
	}
	else {
		r = uOffset.y;
		theta = PI_Over2 - PI_Over4 * (uOffset.x / uOffset.y);
	}
	return r * Vec3(std::cos(theta), std::sin(theta));
}

Vec3 UniformSampleHemisphere(const Vec3& u) {
	double z = u[0];
	double r = std::sqrt(std::max((double)0, (double)1. - z * z));
	double phi = 2 * PI * u[1];
	return Vec3(r * std::cos(phi), r * std::sin(phi), z);
}

double UniformHemispherePdf() { return INV_2PI; }

Vec3 UniformSampleSphere(const Vec3& u) {
	double z = 1 - 2 * u[0];
	double r = std::sqrt(std::max((double)0, (double)1 - z * z));
	double phi = 2 * PI * u[1];
	return Vec3(r * std::cos(phi), r * std::sin(phi), z);
}

double UniformSpherePdf() { return INV_4PI; }

Vec3 UniformSampleCone(const Vec3& u, double cosThetaMax) {
	double cosTheta = ((double)1 - u[0]) + u[0] * cosThetaMax;
	double sinTheta = std::sqrt((double)1 - cosTheta * cosTheta);
	double phi = u[1] * 2 * PI;
	return Vec3(std::cos(phi) * sinTheta, std::sin(phi) * sinTheta,
		cosTheta);
}
Vec3 UniformSampleCone(const Vec3& u, double cosThetaMax, const Vec3& x,
	const Vec3& y, const Vec3& z) {
	double cosTheta = Lerp(u[0], cosThetaMax, 1.f);
	double sinTheta = std::sqrt((double)1. - cosTheta * cosTheta);
	double phi = u[1] * 2 * PI;
	return std::cos(phi) * sinTheta * x + std::sin(phi) * sinTheta * y +
		cosTheta * z;
}

Vec3 CosineSampleHemisphere(const Vec3& u) {
	Vec3 d = ConcentricSampleDisk(u);
	double z = std::sqrt(std::max((double)0, 1 - d.x * d.x - d.y * d.y));
	//std::cout << z << std::endl;
	return Vec3(d.x, d.y, z);
}

double UniformConePdf(double cosThetaMax) {
	return 1 / (2 * PI * (1 - cosThetaMax));
}

Vec3 UniformSampleTriangle(const Vec3& u)
{
	double su0 = std::sqrt(u[0]);
	return Vec3(1 - su0, u[1] * su0, 0);
}


//****************************************************************************//

//
//Vec2 ConcentricSampleDisk(const Vec2& u) {
//	// Map uniform Random numbers to $[-1,1]^2$
//	Vec2 uOffset = 2.0 * u - Vec2(1, 1);
//
//	// Handle degeneracy at the origin
//	if (uOffset.x == 0 && uOffset.y == 0) return Vec2(0, 0);
//
//	// Apply concentric mapping to point
//	double theta, r;
//	if (std::abs(uOffset.x) > std::abs(uOffset.y)) {
//		r = uOffset.x;
//		theta = PI_Over4 * (uOffset.y / uOffset.x);
//	}
//	else {
//		r = uOffset.y;
//		theta = PI_Over2 - PI_Over4 * (uOffset.x / uOffset.y);
//	}
//	return r * Vec2(std::cos(theta), std::sin(theta));
//}
//
//Vec3 UniformSampleSphere(const Vec2& u) {
//	double z = 1 - 2 * u.x;
//	double r = std::sqrt(std::max((double)0, (double)1 - z * z));
//	double phi = 2 * PI * u.y;
//	return Vec3(r * std::cos(phi), r * std::sin(phi), z);
//}
//
//Vec3 CosineSampleHemisphere(const Vec2& u) {
//	Vec2 d = ConcentricSampleDisk(u);
//	double z = std::sqrt(std::max((double)0, 1 - d.x * d.x - d.y * d.y));
//	return Vec3(d.x, d.y, z);
//}
//
//Vec2 UniformSampleTriangle(const Vec2& u) {
//	double su0 = std::sqrt(u[0]);
//	return Vec2(1 - su0, u[1] * su0);
//}
//
//double BalanceHeuristic(int nf, double fPdf, int ng, double gPdf) {
//	return (nf * fPdf) / (nf * fPdf + ng * gPdf);
//}
//
//
//double CosineHemispherePdf(double cosTheta) {
//	return cosTheta * INV_PI;
//}
//
//double UniformSpherePdf() {
//	return INV_4PI;
//}
//
//Vec3 UniformSampleHemisphere(const Vec2& u)
//{
//	double x = std::cos(2 * PI * u.y) * std::sqrt(1 - u.x * u.x);
//	double y = std::sin(2 * PI * u.y) * std::sqrt(1 - u.x * u.x);
//	double z = u.x;
//	return Vec3(x, y, z);
//}
//
//double UniformSampleHemispherePdf()
//{
//	return INV_2PI;
//}
//
//Vec3 UniformSampleCone(const Vec2& u, double cosThetaMax)
//{
//	double cosTheta = ((double)1 - u[0]) + u[0] * cosThetaMax;
//	double sinTheta = std::sqrt((double)1 - cosTheta * cosTheta);
//	double phi = u[1] * 2 * PI;
//	return Vec3(std::cos(phi) * sinTheta, std::sin(phi) * sinTheta, cosTheta);
//}
//
//Vec3 UniformSampleCone(const Vec2& u, double cosThetaMax, const Vec3& x, const Vec3& y, const Vec3& z)
//{
//	double cosTheta = Lerp(u[0], cosThetaMax, (double)1.0);
//	double sinTheta = std::sqrt((double)1. - cosTheta * cosTheta);
//	double phi = u[1] * 2 * PI;
//	return std::cos(phi) * sinTheta * x + std::sin(phi) * sinTheta * y + cosTheta * z;
//}
//
//double UniformConePdf(double cosThetaMax)
//{
//	return 1 / (2 * PI * (1 - cosThetaMax));
//}