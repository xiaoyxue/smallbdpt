#include "Integrator.h"
#include "Linagl.h"
#include "Intersection.h"
#include "Sampler.h"
#include "Scene.h"
#include "VisibilityTester.h"
#include "DebugUtils.h"

Vec3 SimpleDirectIllumination(const Scene& scene, const Intersection& hitPoint, Sampler& sampler)
{
	Vec3 L(0, 0, 0);
	if (!hitPoint.mIsDelta) {
		Light* pLight;
		real pdfLight;
		real pdfA, pdfW;
		Intersection lightPoint;
		pLight = scene.SampleOneLight(&pdfLight, rand());
		Vec3 Le = pLight->Sample(&lightPoint, &pdfA, sampler.Get3D());
		Vec3 hitToLight = lightPoint.mPos - hitPoint.mPos;
		real dis = hitToLight.Length();
		hitToLight.Normalize();
		real cosTheta0 = hitToLight.Dot(hitPoint.mNormal);
		real cosTheta1 = (-1 * hitToLight).Dot(lightPoint.mNormal);
		pdfW = pdfA * dis * dis / std::abs(cosTheta1);
		Vec3 f = hitPoint.mpBSDF->f(hitPoint.mOutDir, hitToLight);
		VisibilityTester visibilityTester(hitPoint, lightPoint);
		Ray shadowRay(hitPoint.mPos, hitToLight);
		Intersection hit;
		if (scene.Intersect(shadowRay, &hit) && cosTheta1 > 0) {
			if (hit.IsLight) {
				L = Le * f * cosTheta0 / pdfW / pdfLight;
			}
		}

	}
	return L;
}
