#ifndef BSDF_H
#define BSDF_H
#include "Geometry.h"
#include "Sampler.h"
#include "Sampling.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include "Utils.h"

inline double CosTheta(const Vec3 &wo) {
	return std::abs(wo.z);
}

inline bool Refract(const Vec3 &wi, const Vec3 &normal, double eta, Vec3 *wt) {
	double cosThetaI = wi.dot(normal);
	double sin2ThetaI = std::max(double(0), double(1 - cosThetaI * cosThetaI));
	double sin2ThetaT = eta * eta * sin2ThetaI;

	if (sin2ThetaT >= 1) return false;
	double cosThetaT = std::sqrt(1 - sin2ThetaT);
	*wt = eta * (-1 * wi) + (eta * cosThetaI - cosThetaT) * normal;
	//*wt = (-1 * normal * cosThetaT) + (cosThetaI * normal - wi).norm() * std::sqrt(sin2ThetaT);
	return true;
}

class BSDF {
public:
	BSDF(){}
	BSDF(Vec3 norm, Vec3 surfacenorm) : ns(norm), surfaceNormal(surfacenorm){
		CoordinateSystem(ns, &ss, &ts);
	}
	virtual Vec3 Sample_f(const Vec3 &wo, Vec3 *wi, double *pdf, const Vec3 &u) const = 0;
	virtual double Pdf(const Vec3 &wo, const Vec3 &wi) const = 0;
	virtual Vec3 f(const Vec3 &wo, const Vec3 &wi) const = 0;
	virtual bool IsDelta() { return false; }
	Vec3 LocalToWorld(const Vec3 &v) const {
		return (ss * v.x + ts * v.y + ns * v.z);
		//return Vec3(ss.x * v.x + ts.x * v.y + ns.x * v.z,
		//	ss.y * v.x + ts.y * v.y + ns.y * v.z,
		//	ss.z * v.x + ts.z * v.y + ns.z * v.z);
	}

	Vec3 WorldToLocal(const Vec3 &v) const {
		return Vec3(v.dot(ss), v.dot(ts), v.dot(ns));
	}

	Vec3 ss, ts, ns, surfaceNormal;
};

class LambertianBSDF : public BSDF {
public:
	LambertianBSDF(){}
	LambertianBSDF(Vec3 normal, Vec3 surfacenorm, Vec3 r) : BSDF(normal, surfacenorm), R(r) {}
	Vec3 Sample_f(const Vec3 &wo, Vec3 *wi, double *pdf, const Vec3 &u) const override {
		Vec3 woLocal = this->WorldToLocal(wo);
		Vec3 wiLocal = CosineSampleHemisphere(u);
		//*pdf = PdfInner(woLocal, wiLocal);
		*wi = this->LocalToWorld(wiLocal);
		*pdf = Pdf(wo, *wi);
		//double r1 = 2 * PI*u[0], r2 = u[1], r2s = sqrt(r2);
		//Vec3 w = ns, uu = ((fabs(w.x)>.1 ? Vec3(0, 1) : Vec3(1)) % w).norm(), v = w%uu;
		//Vec3 d = (uu*cos(r1)*r2s + v*sin(r1)*r2s + w*sqrt(1 - r2)).norm();
		//*wi = d;
		//*pdf = d.dot(ns) * INV_PI;

		return f(wo, *wi);
	}
	Vec3 f(const Vec3 &wo, const Vec3 &wi) const {
		return R * INV_PI;
	}
	double Pdf(const Vec3 &wo, const Vec3 &wi) const {
		Vec3 woLocal = this->WorldToLocal(wo);
		Vec3 wiLocal = this->WorldToLocal(wi);
		return std::abs(wiLocal.z) * INV_PI;
	}
private:
	const Vec3 R;
	double PdfInner(const Vec3 &wo, const Vec3 &wi) const {
		return std::abs(wi.z) * INV_PI;
	}
};

class Fresnel {
public:
	Fresnel(double _eta0 = 1.0, double _eta1 = 1.5) : eta0(_eta0), eta1(_eta1){}
	double Fresnel_Schlick(double InCosTheta, double entering) const {
		if (!entering) {
			InCosTheta = eta1 / eta0 * InCosTheta;
		}
		double R0 = (eta0 - eta1) / (eta0 + eta1);
		R0 = R0 * R0;
		double R = 1 - InCosTheta;
		double R2 = R * R;
		return R0 + (1 - R0) * R * R2 * R2;
	}
	double FrDielectric(double cosThetaI, double etaI, double etaT) const {
		cosThetaI = Clamp(cosThetaI, -1, 1);
		// Potentially swap indices of refraction
		bool entering = cosThetaI > 0.f;
		if (!entering) {
			std::swap(etaI, etaT);
			cosThetaI = std::abs(cosThetaI);
		}

		// Compute _cosThetaT_ using Snell's law
		double sinThetaI = std::sqrt(std::max((double)0, 1 - cosThetaI * cosThetaI));
		double sinThetaT = etaI / etaT * sinThetaI;

		// Handle total internal reflection
		if (sinThetaT >= 1) return 1;
		double cosThetaT = std::sqrt(std::max((double)0, 1 - sinThetaT * sinThetaT));
		double Rparl = ((etaT * cosThetaI) - (etaI * cosThetaT)) /
			((etaT * cosThetaI) + (etaI * cosThetaT));
		double Rperp = ((etaI * cosThetaI) - (etaT * cosThetaT)) /
			((etaI * cosThetaI) + (etaT * cosThetaT));
		return (Rparl * Rparl + Rperp * Rperp) / 2;
	}

private:
	double eta0, eta1;
};


class SpecularReflection : public BSDF {
public:
	SpecularReflection(Vec3 normal, Vec3 surfacenorm, Vec3 r) : BSDF(normal, surfacenorm), R(r) {}
	Vec3 Sample_f(const Vec3 &wo, Vec3 *wi, double *pdf, const Vec3 &u) const override {
		Vec3 woLocal = this->WorldToLocal(wo);
		Vec3 wiLocal(-1 * woLocal.x, -1 * woLocal.y, woLocal.z);
		*wi = this->LocalToWorld(wiLocal);
		*pdf = 1.0;
		return R / wi->dot(ns);
	}
	bool IsDelta() { return true; }
	double Pdf(const Vec3 &wo, const Vec3 &wi) const { return 0.0; }
	Vec3 f(const Vec3 &wo, const Vec3 &wi) const { return Vec3(0.0, 0.0, 0.0); }

private:
	const Vec3 R;
};


class SpecularTransmission : public BSDF {
public:
	SpecularTransmission(Vec3 normal, Vec3 surfacenorm, Vec3 t, double _eta0 = 1.0, double _eta1 = 1.5) :
		BSDF(normal, surfacenorm), T(t), eta0(_eta0), eta1(_eta1), fresnel(_eta0, _eta1){}

	Vec3 Sample_f(const Vec3 &wo, Vec3 *wi, double *pdf, const Vec3 &u) const override {
		Vec3 woLocal = this->WorldToLocal(wo);
		double cosTheta = CosTheta(woLocal);
		double entering = ns.dot(surfaceNormal) > 0;
		double etaI = entering ? eta0 : eta1;
		double etaT = entering ? eta1 : eta0;
		double fresnelreflect = fresnel.FrDielectric(cosTheta, etaI, etaT);
		//double fresnelreflect = fresnel.Fresnel_Schlick(cosTheta, entering);
		double prob = 0.5 * fresnelreflect + 0.25;
		//double prob = 0.0;
		if (u[0] < prob) {
			//reflection
			Vec3 wiLocal(-1 * woLocal.x, -1 * woLocal.y, woLocal.z);
			*wi = this->LocalToWorld(wiLocal);
			*pdf = 1.0 * prob;
			return T / std::abs(CosTheta(wiLocal)) * fresnelreflect;
		}
		else {
			Vec3 wiLocal;
			Vec3 n(0, 0, 1);
			n = n.dot(woLocal) < 0 ? -1 * n : n;
			if (!Refract(woLocal, n, etaI / etaT, &wiLocal)) {
				//std::cout << "internel reflection" << std::endl;
				/*
				wiLocal = Vec3(-1 * woLocal.x, -1 * woLocal.y, woLocal.z);
				*wi = this->LocalToWorld(wiLocal);
				*pdf = 1.0 * (1 - prob);
				return T / wi->dot(ns) * (1 - fresnelreflect);*/
				return Vec3(0, 0, 0);
			}
			else {
				*pdf = 1.0 * (1 - prob);
				*wi = this->LocalToWorld(wiLocal);
				return (1 - fresnelreflect) * T * (etaI * etaI) / (etaT * etaT) / std::abs(CosTheta(wiLocal));
			}
		}
		return Vec3(0.0, 0.0, 0.0);
	}

	bool IsDelta() { return true; }
	double Pdf(const Vec3 &wo, const Vec3 &wi) const { return 0.0; }
	Vec3 f(const Vec3 &wo, const Vec3 &wi) const { return Vec3(0.0, 0.0, 0.0); }
private:
	const Vec3 T;
	const double eta0, eta1;
	const Fresnel fresnel;
};

#endif
