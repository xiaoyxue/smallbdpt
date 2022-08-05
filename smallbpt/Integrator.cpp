#include "Integrator.h"
#include "Linagl.h"
#include "Intersection.h"
#include "Sampler.h"
#include "Scene.h"

Vec3 SimpleDirectIllumination(const Scene& scene, const Intersection& hitPoint, Sampler& sampler)
{
	Vec3 L(0, 0, 0);
	if (!hitPoint.Delta) {
		Light* pLight;
		double pdfLight;
		double pdfA, pdfW;
		Intersection lightPoint;
		pLight = scene.SampleOneLight(&pdfLight, sampler.Get1D());
		Vec3 Le = pLight->Sample(&lightPoint, &pdfA, sampler.Get3D());
		Vec3 hitToLight = lightPoint.HitPoint - hitPoint.HitPoint;
		double dis = hitToLight.Length();
		hitToLight = hitToLight.Norm();
		double cosTheta0 = hitToLight.Dot(hitPoint.Normal);
		double cosTheta1 = (-1 * hitToLight).Dot(lightPoint.Normal);
		pdfW = pdfA * dis * dis / std::abs(cosTheta1);
		Vec3 f = hitPoint.bsdf->f(hitPoint.wo, hitToLight);
		Ray shadowRay(hitPoint.HitPoint, hitToLight);
		double t;
		Intersection isect;
		scene.Intersect(shadowRay, t, isect);
		if (!isect.IsLight || cosTheta1 < 0) {
			return Vec3(0, 0, 0);
		}
		else {
			L = Le * f * std::abs(cosTheta0) / pdfW / pdfLight;
		}
	}
	return L;
}
