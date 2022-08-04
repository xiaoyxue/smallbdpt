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
	Vec3 lightNormal = (pos - mpSphere->p).norm();
	Vec3 ss, ts;
	CoordinateSystem(lightNormal, &ss, &ts);
	Vec3 dirLocal = CosineSampleHemisphere(sampler.Get3D());
	*cosTheta = dirLocal.z;
	*dir = (ss * dirLocal.x + ts * dirLocal.y + lightNormal * dirLocal.z).norm();
	*pdfDir = CosineHemispherePdf(*cosTheta);
	lightPoint->HitPoint = pos;
	lightPoint->Normal = lightNormal;
	lightPoint->SurfaceNormal = lightNormal;
	return mpSphere->Emission();
}

Vec3 SphereLight::DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3 &throughput, PathVertex *sampled) const
{
	Vec3 L;
	Vec3 localZ = (mpSphere->p - isect.HitPoint).norm(), localX, localY;
	CoordinateSystem(localZ, &localX, &localY);

	double SinThetaMax = mpSphere->rad / (mpSphere->p - isect.HitPoint).length();
	double CosThetaMax = std::sqrt(1 - SinThetaMax * SinThetaMax);
	Vec3 wi = UniformSampleCone(sampler.Get3D(), CosThetaMax, localX, localY, localZ);
	double PdfW = UniformConePdf(CosThetaMax);

	//calculate the hit point and normal
	double CosTheta = wi.dot(localZ);
	double SinTheta = std::sqrt(std::max(0.0, 1 - CosTheta * CosTheta));
	double dc = (mpSphere->p - isect.HitPoint).length();
	double ds = dc * CosTheta - std::sqrt(std::max(0.0, mpSphere->rad * mpSphere->rad - dc * dc * SinTheta * SinTheta));
	Vec3 HitPoint = isect.HitPoint + ds * wi;
	Vec3 HitNormal = (HitPoint - mpSphere->p).norm();

	Ray shadowRay(isect.HitPoint, wi);
	double t; Intersection isection;
	Vec3 f = isect.bsdf->f(isect.wo, wi);
	if (!scene.Intersect(shadowRay, t, isection) || !isection.IsLight) L = Vec3(0.0, 0.0, 0.0);
	else L = throughput * f * wi.dot(isect.Normal) * mpSphere->Emission() / PdfW;

	if(sampled) {
		sampled->mThroughput = mpSphere->Emission() / PdfW;
		sampled->isect.HitPoint = HitPoint;
		sampled->isect.Normal = HitNormal;
	}


	return L;
}
