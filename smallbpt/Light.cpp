#include "Light.h"
#include "PathVertex.h"
#include "Scene.h"


Vec3 SphereLight::Sample(Intersection* lightPoint, double* pdf, const Vec3& u) const
{
	*lightPoint = mpSphere->Sample(pdf, u);
	return Emission();
}

Vec3 SphereLight::SampleFromLight(Intersection* lightPoint, Vec3* dir, double* pdfPos, double* pdfDir, double *cosTheta, Sampler& sampler) const
{
	Vec3 pos = UniformSampleSphere(sampler.Get3D()) * mpSphere->rad + mpSphere->p;
	*pdfPos = 1.f / (4 * PI * mpSphere->rad * mpSphere->rad);
	Vec3 lightNormal = (pos - mpSphere->p).Norm();
	Vec3 ss, ts;
	CoordinateSystem(lightNormal, &ss, &ts);
	Vec3 dirLocal = CosineSampleHemisphere(sampler.Get3D());
	*cosTheta = dirLocal.z;
	*dir = (ss * dirLocal.x + ts * dirLocal.y + lightNormal * dirLocal.z).Norm();
	*pdfDir = CosineHemispherePdf(*cosTheta);
	lightPoint->HitPoint = pos;
	lightPoint->Normal = lightNormal;
	lightPoint->SurfaceNormal = lightNormal;
	return mpSphere->Emission();
}

Vec3 SphereLight::DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3 &throughput, PathVertex *sampled) const
{
	Vec3 L;
	Vec3 localZ = (mpSphere->p - isect.HitPoint).Norm(), localX, localY;
	CoordinateSystem(localZ, &localX, &localY);

	double SinThetaMax = mpSphere->rad / (mpSphere->p - isect.HitPoint).Length();
	double CosThetaMax = std::sqrt(1 - SinThetaMax * SinThetaMax);
	Vec3 wi = UniformSampleCone(sampler.Get3D(), CosThetaMax, localX, localY, localZ);
	double PdfW = UniformConePdf(CosThetaMax);

	//calculate the hit point and normal
	double CosTheta = wi.Dot(localZ);
	double SinTheta = std::sqrt(std::max(0.0, 1 - CosTheta * CosTheta));
	double dc = (mpSphere->p - isect.HitPoint).Length();
	double ds = dc * CosTheta - std::sqrt(std::max(0.0, mpSphere->rad * mpSphere->rad - dc * dc * SinTheta * SinTheta));
	Vec3 HitPoint = isect.HitPoint + ds * wi;
	Vec3 HitNormal = (HitPoint - mpSphere->p).Norm();

	Ray shadowRay(isect.HitPoint, wi);
	Intersection isection;
	Vec3 f = isect.bsdf->f(isect.wo, wi);
	if (!scene.Intersect(shadowRay, &isection) || !isection.IsLight) L = Vec3(0.0, 0.0, 0.0);
	else L = throughput * f * std::abs(wi.Dot(isect.Normal)) * Emission() / PdfW;

	if(sampled) {
		sampled->mThroughput = mpSphere->Emission() / PdfW;
		sampled->mIsect.HitPoint = HitPoint;
		sampled->mIsect.Normal = HitNormal;
	}

	return L;
}

Vec3 AreaLight::Sample(Intersection* lightPoint, double* pdf, const Vec3& u) const
{
	*lightPoint = mpShape->Sample(pdf, u);
	return Emission();
}

Vec3 AreaLight::SampleFromLight(Intersection* lightPoint, Vec3* dir, double* pdfPos, double* pdfDir, double* cosTheta, Sampler& sampler) const
{
	*lightPoint = mpShape->Sample(pdfPos, sampler.Get3D());
	Vec3 ss, ts;
	CoordinateSystem(lightPoint->Normal, &ss, &ts);
	Vec3 dirLocal = CosineSampleHemisphere(sampler.Get3D());
	*cosTheta = dirLocal.z;
	*dir = (ss * dirLocal.x + ts * dirLocal.y + lightPoint->Normal * dirLocal.z).Norm();
	*pdfDir = CosineHemispherePdf(*cosTheta);

	return Emission();
}

Vec3 AreaLight::DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3& throughput, PathVertex* sampled /*= 0*/) const
{
	Vec3 L;
	double pdfA, pdfW;
	Intersection lightPoint = mpShape->Sample(&pdfA, sampler.Get3D());
	Vec3 wi = (lightPoint.HitPoint - isect.HitPoint);
	double dis = wi.Length();
	double dis2 = dis * dis;
	wi = wi.Norm();
	pdfW = pdfA * dis2 / std::abs(lightPoint.Normal.Dot(-1 * wi));
	Vec3 f = isect.bsdf->f(isect.wo, wi);
	Ray shadowRay(isect.HitPoint, wi);
	Intersection isection;
	scene.Intersect(shadowRay, &isection);
	if (!isection.IsLight) {
		return Vec3();
	}
	else {
		L = throughput * f * std::abs(wi.Dot(isect.Normal)) * Emission() / pdfW;
	}

	if (sampled) {
		sampled->mThroughput = mpShape->Emission() / pdfW;
		sampled->mIsect.HitPoint = lightPoint.HitPoint;
		sampled->mIsect.Normal = lightPoint.HitPoint;
	}

	return L;

}
