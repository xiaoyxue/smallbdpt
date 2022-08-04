//#include "LightTracing.h"
//#include "Scene.h"
//#include "Sampling.h"
//#include "Intersection.h"
//#include "iostream"
//#include "Smallbpt.h"
//#include "Utils.h"
//
//void LightTracing::Render(const Scene &scene, const Camera &camera) {
//	int64_t resX = camera.GetFilm()->resX;
//	int64_t resY = camera.GetFilm()->resY;
//	int nTotalSamples = spp * resX * resY;
////#pragma omp parallel for schedule(dynamic, 1)
//	for (int p = 0; p < nTotalSamples; ++p) {
//		int nVertex = GenerateLightPath(scene, *sampler, LightPath, maxDepth);
//		for (int s = 0; s < nVertex; ++s) {
//			Vec3 pRaster;
//			bool inScreen;
//			Vec3 L = ConnectToCamera(scene, LightPath[s], s, camera, &pRaster, &inScreen);
//			if (inScreen) {
//				//L = L * camera->film->Area / NtotalSample;
//				L = L * resX * resY / nTotalSamples;
//				//L = L / spp;
//				L = Vec3(clamp(L.x), clamp(L.y), clamp(L.z));
//				camera.GetFilm()->AddSplat(pRaster, L);
//			}
//		}
//	}
//	camera.GetFilm()->WriteToImage();
//	
//}
//
//
//int LightTracing::GenerateLightPath(const Scene &scene, Sampler &sampler, std::vector<PathVertex> &lightPath, int maxdepth) {
//	Sphere light = Scene::spheres[Scene::numSpheres - 1];
//	Vec3 Le = light.e;
//	//sample a position
//	Vec3 pos = UniformSampleSphere(sampler.Get3D()) * light.rad + light.p;
//	double Pdfpos = 1.f / (4 * PI * light.rad * light.rad);
//	Vec3 lightNorm = (pos - light.p).norm();
//	Vec3 ss, ts;
//	CoordinateSystem(lightNorm, &ss, &ts);
//	Vec3 dirLocal = CosineSampleHemisphere(sampler.Get3D());
//	double CosTheta = dirLocal.z;
//	Vec3 lightDir = (ss * dirLocal.x + ts * dirLocal.y + lightNorm * dirLocal.z).norm();
//	double Pdfdir = CosineHemispherePdf(CosTheta);
//	Ray ray(pos + lightDir * eps, lightDir);
//	lightPath[0].isect.HitPoint = pos;
//	lightPath[0].isect.Normal = (pos - light.p).norm();
//	lightPath[0].mThroughput = Le; // / Pdfdir / Pdfpos * CosTheta;
//	lightPath[0].mPdfFwd = Pdfpos;
//	lightPath[0].isect.Delta = false;
//	Vec3 Throughput = lightPath[0].mThroughput * CosTheta / Pdfpos / Pdfdir;
//
//#ifdef _DEBUG
//	//std::cout << LightPath[0].Throughput.x << " " << LightPath[0].Throughput.y << " " << LightPath[0].Throughput.z << std::endl;
//#endif
//
//	return Trace(scene, ray, Throughput, Pdfdir, sampler, lightPath, 1, maxdepth);
//}
//
//int LightTracing::Trace(const Scene &scene, const Ray &ray, Vec3 Throughput, double PdfFwd, Sampler &sampler, std::vector<PathVertex> &LightPath, int depth, int maxDepth) {
//	double pdfdir = PdfFwd;
//	Ray r = ray;
//	int bound = depth;
//	double PdfW = PdfFwd;
//	while (1) {
//		Intersection &isect = LightPath[bound].isect;
//		double t;
//		int id;
//		if (!scene.intersect(r, t, id, isect)) break;
//#ifdef _DEBUG
//		//std::cout << id << std::endl;
//#endif
//		LightPath[bound].mThroughput = Throughput;
//		LightPath[bound].mPdfFwd = PdfW;
//#ifdef _DEBUG
//		//std::cout << Throughput.x << " " << Throughput.y << " " << Throughput.z << std::endl;
//#endif
//		++bound;
//		if (bound >= maxDepth + 1) break;
//
//		Vec3 wo;
//		Vec3 f = isect.bsdf->Sample_f(-1 * r.d, &wo, &PdfW, sampler.Get3D());
//		wo.norm();
//		Throughput = Throughput * f * (wo.dot(isect.Normal)) / PdfW;
//
//		r.o = isect.HitPoint + wo * eps;
//		r.d = wo;
//
//	}
//
//	return bound - 1;
//}
//
//
//Vec3 LightTracing::ConnectToCamera(const Scene &scene, const PathVertex &Vertex, int s, const Camera &camera, Vec3 *pRaster, bool *inScreen) {
//	if (Vertex.isect.Delta) return Vec3(0.0, 0.0, 0.0);
//
//	*inScreen = true;
//	Vec3 dirHitPointToCam = camera.o - Vertex.isect.HitPoint;
//	double ray_tmax = dirHitPointToCam.length();
//	dirHitPointToCam.norm();
//	if (camera.d.dot(-1 * dirHitPointToCam) < 0) {
//		*inScreen = false;
//		return Vec3(0.0, 0.0, 0.0);
//	}
//
//	bool isInScreen;
//	*pRaster = camera.WordToScreen(Vertex.isect.HitPoint, &isInScreen);
//	if (!isInScreen) {
//		*inScreen = false;
//		return Vec3(0.0, 0.0, 0.0);
//	}
//	
//	Ray ray(Vertex.isect.HitPoint, dirHitPointToCam, 0.f, ray_tmax);
//	Intersection isc;
//	double t; int id;
//	if (scene.intersect(ray)){
//		*inScreen = false;
//		return Vec3(0.0, 0.0, 0.0);
//	}
//	if (s == 0) {
//		return Vertex.mThroughput; //see the light source directly
//	}
//	double PdfW;
//	Vec3 wi;
//	Vec3 We = camera.Sample_Wi(Vertex.isect, &PdfW, &wi);
//	Vec3 f = Vertex.isect.bsdf->f(Vertex.isect.wo, wi);
//	Vec3 L = We * Vertex.mThroughput * f * wi.dot(Vertex.isect.Normal) / PdfW;
//	return L;
//} 