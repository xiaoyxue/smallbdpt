#ifndef SAMPLING_H
#define SAMPLING_H
#include "Geometry.h"
#include <algorithm>

struct Distribution1D {
	// Distribution1D Public Methods
	Distribution1D(const double* f, int n) : func(f, f + n), cdf(n + 1) {
		// Compute integral of step function at $x_i$
		cdf[0] = 0;
		for (int i = 1; i < n + 1; ++i) cdf[i] = cdf[i - 1] + func[i - 1] / n;

		// Transform step function integral into CDF
		funcInt = cdf[n];
		if (funcInt == 0) {
			for (int i = 1; i < n + 1; ++i) cdf[i] = double(i) / double(n);
		}
		else {
			for (int i = 1; i < n + 1; ++i) cdf[i] /= funcInt;
		}
	}
	int Count() const { return (int)func.size(); }
	double SampleContinuous(double u, double* pdf, int* off = nullptr) const {
		// Find surrounding CDF segments and _offset_
		int offset = FindInterval((int)cdf.size(),
			[&](int index) { return cdf[index] <= u; });
		if (off) *off = offset;
		// Compute offset along CDF segment
		double du = u - cdf[offset];
		if ((cdf[offset + 1] - cdf[offset]) > 0) {
			du /= (cdf[offset + 1] - cdf[offset]);
		}
		// Compute PDF for sampled offset
		if (pdf) *pdf = (funcInt > 0) ? func[offset] / funcInt : 0;

		// Return $x\in{}[0,1)$ corresponding to sample
		return (offset + du) / Count();
	}
	int SampleDiscrete(double u, double* pdf = nullptr,
		double* uRemapped = nullptr) const {
		// Find surrounding CDF segments and _offset_
		int offset = FindInterval((int)cdf.size(),
			[&](int index) { return cdf[index] <= u; });
		if (pdf) *pdf = (funcInt > 0) ? func[offset] / (funcInt * Count()) : 0;
		if (uRemapped)
			*uRemapped = (u - cdf[offset]) / (cdf[offset + 1] - cdf[offset]);
		return offset;
	}
	double DiscretePDF(int index) const {
		return func[index] / (funcInt * Count());
	}

	// Distribution1D Public Data
	std::vector<double> func, cdf;
	double funcInt;
};

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