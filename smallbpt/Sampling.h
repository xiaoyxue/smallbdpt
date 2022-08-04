#ifndef SAMPLING_H
#define SAMPLING_H
#include "Geometry.h"
#include <algorithm>

Vec3 UniformSampleDisk(const Vec3& u);

Vec3 ConcentricSampleDisk(const Vec3& u);

Vec3 UniformSampleHemisphere(const Vec3& u);

Vec3 UniformSampleSphere(const Vec3& u);

double UniformHemispherePdf();

double UniformSpherePdf();

Vec3 UniformSampleCone(const Vec3& u, double cosThetaMax);

Vec3 UniformSampleCone(const Vec3& u, double cosThetaMax, const Vec3& x, const Vec3& y, const Vec3& z);

double UniformConePdf(double cosThetaMax);

Vec3 CosineSampleHemisphere(const Vec3& u);

inline double CosineHemispherePdf(double cosTheta) { return cosTheta * INV_PI; }

Vec3 UniformSampleTriangle(const Vec3& u);

//***************************************************************************************************

//Vec2 ConcentricSampleDisk(const Vec2& u);
//
//Vec3 UniformSampleSphere(const Vec2& u);
//
//Vec3 CosineSampleHemisphere(const Vec2& u);
//
//double UniformSpherePdf();
//
//Vec2 UniformSampleTriangle(const Vec2& u);
//
//double BalanceHeuristic(int nf, double fPdf, int ng, double gPdf);
//
//double CosineHemispherePdf(double cosTheta);
//
//Vec3 UniformSampleHemisphere(const Vec2& u);
//
//double UniformSampleHemispherePdf();
//
//Vec3 UniformSampleCone(const Vec2& u, double cosThetaMax);
//
//Vec3 UniformSampleCone(const Vec2& u, double cosThetaMax, const Vec3& x, const Vec3& y, const Vec3& z);
//
//double UniformConePdf(double cosThetaMax);

#endif