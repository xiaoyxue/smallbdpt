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
	lightPoint->mPos = pos;
	lightPoint->mNormal = lightNormal;
	lightPoint->mSurfaceNormal = lightNormal;
	return mpSphere->Emission();
}

Vec3 SphereLight::DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3 &throughput, PathVertex *sampled) const
{
	Vec3 L;
	Vec3 localZ = (mpSphere->p - isect.mPos).Norm(), localX, localY;
	CoordinateSystem(localZ, &localX, &localY);

	double SinThetaMax = mpSphere->rad / (mpSphere->p - isect.mPos).Length();
	double CosThetaMax = std::sqrt(1 - SinThetaMax * SinThetaMax);
	Vec3 wi = UniformSampleCone(sampler.Get3D(), CosThetaMax, localX, localY, localZ);
	double PdfW = UniformConePdf(CosThetaMax);

	//calculate the hit point and normal
	double CosTheta = wi.Dot(localZ);
	double SinTheta = std::sqrt(std::max(0.0, 1 - CosTheta * CosTheta));
	double dc = (mpSphere->p - isect.mPos).Length();
	double ds = dc * CosTheta - std::sqrt(std::max(0.0, mpSphere->rad * mpSphere->rad - dc * dc * SinTheta * SinTheta));
	Vec3 HitPoint = isect.mPos + ds * wi;
	Vec3 HitNormal = (HitPoint - mpSphere->p).Norm();

	Ray shadowRay(isect.mPos, wi);
	Intersection isection;
	Vec3 f = isect.mpBSDF->f(isect.mOutDir, wi);
	if (!scene.Intersect(shadowRay, &isection) || !isection.IsLight) L = Vec3(0.0, 0.0, 0.0);
	else L = throughput * f * std::abs(wi.Dot(isect.mNormal)) * Emission() / PdfW;

	if(sampled) {
		sampled->mThroughput = mpSphere->Emission() / PdfW;
		sampled->mIsect.mPos = HitPoint;
		sampled->mIsect.mNormal = HitNormal;
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
	CoordinateSystem(lightPoint->mNormal, &ss, &ts);
	Vec3 dirLocal = CosineSampleHemisphere(sampler.Get3D());
	*cosTheta = dirLocal.z;
	*dir = (ss * dirLocal.x + ts * dirLocal.y + lightPoint->mNormal * dirLocal.z).Norm();
	*pdfDir = CosineHemispherePdf(*cosTheta);

	return Emission();
}

Vec3 AreaLight::DirectIllumination(const Scene& scene, Sampler& sampler, const Intersection& isect, const Vec3& throughput, PathVertex* sampled /*= 0*/) const
{
	Vec3 L(0, 0, 0);
	if (!isect.mIsDelta) {
		Light* pLight;
		real pdfLight;
		real pdfA, pdfW;
		Intersection lightPoint;
		pLight = scene.SampleOneLight(&pdfLight, rand());
		Vec3 Le = pLight->Sample(&lightPoint, &pdfA, sampler.Get3D());
		Vec3 hitToLight = lightPoint.mPos - isect.mPos;
		real dis = hitToLight.Length();
		hitToLight.Normalize();
		real cosTheta0 = hitToLight.Dot(isect.mNormal);
		real cosTheta1 = (-1 * hitToLight).Dot(lightPoint.mNormal);
		pdfW = pdfA * dis * dis / std::abs(cosTheta1);
		Vec3 f = isect.mpBSDF->f(isect.mOutDir, hitToLight);
		Ray shadowRay(isect.mPos, hitToLight);
		Intersection hit;
		if (scene.Intersect(shadowRay, &hit) && cosTheta1 > 0) {
			if (hit.IsLight) {
				L = throughput * f * cosTheta0 * Emission() / pdfW / pdfLight;
			}
		}
		if (sampled) {
			sampled->mThroughput = mpShape->Emission() / pdfW;
			sampled->mIsect.mPos = isect.mPos;
			sampled->mIsect.mNormal = isect.mPos;
		}
	}
	return L;

}
