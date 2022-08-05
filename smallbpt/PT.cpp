#include "PT.h"
#include "Intersection.h"
#include "Scene.h"

Vec3 PathTracing::Li(const Scene& scene, const Ray& r) const
{
	Ray ray = r;
	Vec3 L(0, 0, 0);
	bool deltaBoundEvent = false;
	Vec3 throughput = Vec3(1, 1, 1);
	for (int i = 0; i < mMaxDepth; ++i) {
		Intersection isect;
		double t;
		if (!scene.Intersect(ray, t, isect)) {
			break;
		}

		std::shared_ptr<BSDF> bsdf = isect.bsdf;

		if ((i == 0 || deltaBoundEvent) && isect.IsLight) {
			Shape* pLight = isect.pLight;
			L += throughput * pLight->Emission();
		}
		else {
			L += throughput * SimpleDirectIllumination(scene, isect, *mpSampler);
		}
		Vec3 wi;
		double pdfW;
		Vec3 f = bsdf->Sample_f(-1 * ray.d, &wi, &pdfW, mpSampler->Get3D());
		if (f == Vec3() || pdfW == 0) break;
		Vec3 estimation = f * std::abs(isect.Normal.Dot(wi)) / pdfW;
		deltaBoundEvent = bsdf->IsDelta();

		throughput = throughput * estimation;
		ray = Ray(isect.HitPoint, wi);
	}
	return L;
}
