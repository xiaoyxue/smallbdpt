#include "PT.h"
#include "Intersection.h"
#include "Scene.h"

Vec3 PathTracing::Li(const Ray& r, const Scene& scene, Sampler& sampler) const
{
	Ray ray = r;
	Vec3 L(0, 0, 0);
	bool deltaBoundEvent = false;
	Vec3 throughput = Vec3(1, 1, 1);
	for (int i = 0; i < mMaxDepth; ++i) {
		Intersection isect;
		if (!scene.Intersect(ray, &isect)) {
			break;
		}

		std::shared_ptr<BSDF> pBSDF = isect.mpBSDF;

		if ((i == 0 || deltaBoundEvent) && isect.IsLight) {
			Shape* pLight = isect.pLight;
			L += throughput * pLight->Emission();
		}
		else {
			L += throughput * SimpleDirectIllumination(scene, isect, sampler);
		}
		Vec3 wi;
		double pdfW;
		Vec3 f = pBSDF->Sample_f(-1 * ray.d, &wi, &pdfW, sampler.Get3D());
		if (f == Vec3() || pdfW == 0) break;
		Vec3 estimation = f * std::abs(isect.mNormal.Dot(wi)) / pdfW;
		deltaBoundEvent = pBSDF->IsDelta();

		double p = std::min(1.0, (estimation * throughput).Y() / throughput.Y());
		if (p < 1 && i > 5) {
			if (sampler.Get1D() < p) {
				throughput = throughput / p;
			}
			else {
				break;
			}
		}

		throughput = throughput * estimation;
		ray = Ray(isect.mPos, wi);
	}
	return L;
}
