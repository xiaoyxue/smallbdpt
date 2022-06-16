#include "BDPT.h"
#include "PathTracing.h"
#include "Scene.h"
#include "LightTracing.h"
#include "Sampling.h"
#include "string"

int GenerateLightPath(Sampler &sampler, std::vector<PathVertex> &LightPath, int maxdepth) {
	if (maxdepth == 0) return 0;
	Sphere light = spheres[numSpheres - 1];
	Vec3 Le = light.e;
	//sample a position
	Vec3 pos = UniformSampleSphere(sampler.Get2D()) * light.rad + light.p;
	double Pdfpos = 1.f / (4 * PI * light.rad * light.rad);
	Vec3 lightNorm = (pos - light.p).norm();
	Vec3 ss, ts;
	CoordinateSystem(lightNorm, &ss, &ts);
	Vec3 dirLocal = CosineSampleHemisphere(sampler.Get2D());
	double CosTheta = dirLocal.z;
	Vec3 lightDir = (ss * dirLocal.x + ts * dirLocal.y + lightNorm * dirLocal.z).norm();
	double Pdfdir = CosineHemispherePdf(CosTheta);
	Ray ray(pos + lightDir * eps, lightDir);
	LightPath[0].isect.HitPoint = pos;
	LightPath[0].isect.Normal = (pos - light.p).norm();
	LightPath[0].Throughput = Le; // / Pdfdir / Pdfpos * CosTheta;
	LightPath[0].PdfFwd = Pdfpos;
	LightPath[0].isect.Delta = false;
	LightPath[0].isect.IsLight = true;
	Vec3 Throughput = LightPath[0].Throughput * CosTheta / Pdfpos / Pdfdir;
	return Trace(ray, Throughput, Pdfdir, sampler, LightPath, 1, maxdepth - 1);
}

int GenerateCameraPath(Sampler &sampler, std::vector<PathVertex> &CameraPath,
	const Camera &camera, const Ray &cameraRay, int maxdepth) {
	if (maxdepth == 0) return 0;
	Vec3 Throughput(1.0, 1.0, 1.0);
	// Throughtput = We * CosTheta / Pdfpos / PdfW = (1, 1, 1)
	CameraPath[0].isect.HitPoint = camera.o;
	CameraPath[0].isect.Normal = camera.d;
	CameraPath[0].PdfFwd = camera.PdfPos();
	CameraPath[0].Throughput = Throughput;
	double Pdfdir = camera.PdfDir(cameraRay);
	const Camera *cam = &camera;
	CameraPath[0].camera = const_cast<Camera*>(cam);

	return Trace(cameraRay, Throughput, Pdfdir, sampler, CameraPath, 1, maxdepth - 1);
}


int Trace(const Ray &ray, Vec3 Throughput, double PdfFwd, Sampler &sampler,
	std::vector<PathVertex> &Path, int depth, int maxDepth) {
	Ray r = ray;
	int bound = depth;
	double PdfW = PdfFwd;
	while (1) {
		PathVertex &prev = Path[bound - 1];
		PathVertex &vertex = Path[bound];
		Intersect &isect = Path[bound].isect;
		double t;
		int id;
		if (!intersect(r, t, id, isect)) break;

		
		Path[bound].Throughput = Throughput;
		Path[bound].PdfFwd = ConvertSolidToArea(PdfW, prev, vertex);
 		++bound;
		if (bound >= maxDepth + 1) break;

		Vec3 wo;
		Vec3 f = isect.bsdf->Sample_f(-1 * r.d, &wo, &PdfW, sampler.Get2D());
		wo.norm();
		Throughput = Throughput * f * (std::abs(wo.dot(isect.Normal))) / PdfW;

		double PdfWPrev = isect.bsdf->Pdf(wo, -1 * r.d);
		prev.PdfPrev = ConvertSolidToArea(PdfWPrev, vertex, prev);
		
		if (isect.Delta) {
			PdfW = PdfWPrev = 0;
		}

		r.o = isect.HitPoint;// +wo * eps;
		r.d = wo;
	}
	return bound;
}


double ConvertSolidToArea(double PdfW, const PathVertex &Vertex, const PathVertex &nxt) {
	Vec3 dir = nxt.isect.HitPoint - Vertex.isect.HitPoint;
	double dist = dir.length();
	double dist2 = dist * dist;
	if (dist2 == 0) return 0;
	dir.norm();
	double CosTheta = std::abs(dir.dot(nxt.isect.Normal));
	return PdfW * CosTheta / dist2;
}

bool IsConnectible(const Vec3 &PointA, const Vec3 &PointB) {
	Vec3 dir = PointA - PointB;
	double dist = dir.length();
	Ray ray(PointB, dir.norm(), 0.0, dist - eps);  //Be careful ! Float error will cause the uncorrect intersection
	
#ifdef _DEBUG
	int id;
	if (intersect(ray, id)) return false;
#else
	if (intersect(ray)) return false;
#endif
	return true;
}

double G(const PathVertex &VertexA, const PathVertex &VertexB) {
	Vec3 dirAtoB = VertexB.isect.HitPoint - VertexA.isect.HitPoint;
	double dist = dirAtoB.length();
	dirAtoB.norm();
	double CosThetaA = std::abs(dirAtoB.dot(VertexA.isect.Normal));
	double CosThetaB = std::abs(dirAtoB.dot(VertexB.isect.Normal));
	double g = CosThetaA * CosThetaB / dist / dist;
	return g;
}


double MISWeight(std::vector<PathVertex> &LightPath, std::vector<PathVertex> &CameraPath,
	int s, int t, PathVertex &sampled) {
	if (s + t == 2) return 1;

	PathVertex *LightVertex = s > 0 ? &LightPath[s - 1] : nullptr,
		*CameraVertex = t > 0 ? &CameraPath[t - 1] : nullptr,
		*LightVertexMinus = s > 1 ? &LightPath[s - 2] : nullptr,
		*CameraVertexMinus = t > 1 ? &CameraPath[t - 2] : nullptr;

	ScopedAssignment<PathVertex> a1;
	if (s == 1)
		a1 = { LightVertex, sampled };
	else if (t == 1)
		a1 = { CameraVertex, sampled };


	ScopedAssignment<bool> a2, a3;
	if (LightVertex) a2 = { &LightVertex->isect.Delta, false };
	if (CameraVertex) a3 = { &CameraVertex->isect.Delta, false };

	ScopedAssignment<double> a4;
	if (CameraVertex) {
		if (s > 0) {
			double PdfW, PdfA;
			if (LightVertexMinus == nullptr) {
				Vec3 LightToHitPoint = CameraVertex->isect.HitPoint - LightVertex->isect.HitPoint;
				LightToHitPoint.norm();
				double CosTheta = LightVertex->isect.Normal.dot(LightToHitPoint);
				PdfW = CosineHemispherePdf(CosTheta);
				PdfA = ConvertSolidToArea(PdfW, *LightVertex, *CameraVertex);
			}
			else {
				Vec3 wo = (LightVertexMinus->isect.HitPoint - LightVertex->isect.HitPoint).norm();
				Vec3 wi = (CameraVertex->isect.HitPoint - LightVertex->isect.HitPoint).norm();
				PdfW = LightVertex->isect.bsdf->Pdf(wo, wi);
				PdfA = ConvertSolidToArea(PdfW, *LightVertex, *CameraVertex);
			}
			a4 = { &CameraVertex->PdfPrev, PdfA };
		}
		else {
			const Sphere &light = spheres[numSpheres - 1];
			double r = light.rad;
			double Area = 4 * PI * r * r;
			double Pdfpos = 1.0 / Area;
			a4 = { &CameraVertex->PdfPrev, Pdfpos };
		}
	}

	ScopedAssignment<double> a5;
	if (CameraVertexMinus) {
		if (s > 0) {
			Vec3 wo = (LightVertex->isect.HitPoint - CameraVertex->isect.HitPoint).norm();
			Vec3 wi = (CameraVertexMinus->isect.HitPoint - CameraVertex->isect.HitPoint).norm();
			double PdfW = CameraVertex->isect.bsdf->Pdf(wo, wi);
			double PdfA = ConvertSolidToArea(PdfW, *CameraVertex, *CameraVertexMinus);
			a5 = { &CameraVertexMinus->PdfPrev, PdfA };
		}
		else {
			const Sphere &light = spheres[numSpheres - 1];
			Vec3 HitPointToLight = CameraVertex->isect.HitPoint - CameraVertexMinus->isect.HitPoint;
			double dist = HitPointToLight.length();
			HitPointToLight.norm();
			Vec3 LightToHitPoint = -1 * HitPointToLight;
			Vec3 LightNormal = CameraVertex->isect.Normal;
			double CosThetaLight = std::abs(LightNormal.dot(LightToHitPoint));
			double PdfW = CosineHemispherePdf(CosThetaLight);
			double PdfA = PdfW * (std::abs(CameraVertexMinus->isect.Normal.dot(HitPointToLight))) / dist / dist;
			a5 = { &CameraVertexMinus->PdfPrev, PdfA };
		}

	}

	// Update reverse density of vertices $\pq{}_{s-1}$ and $\pq{}_{s-2}$
	ScopedAssignment<double> a6;
	if (LightVertex) {
		double PdfW, PdfA;
		if (CameraVertexMinus == nullptr) {
			Vec3 CameraToHitPoint = LightVertex->isect.HitPoint - CameraVertex->isect.HitPoint;
			CameraToHitPoint.norm();
			double CosTheata = std::abs(CameraVertex->isect.Normal.dot(CameraToHitPoint));
			Ray ray(CameraVertex->isect.HitPoint, CameraToHitPoint);
			PdfW = CameraVertex->camera->PdfDir(ray);
			PdfA = ConvertSolidToArea(PdfW, *CameraVertex, *LightVertex);
		}
		else {
			Vec3 wo = (CameraVertexMinus->isect.HitPoint - CameraVertex->isect.HitPoint).norm();
			Vec3 wi = (LightVertex->isect.HitPoint - CameraVertex->isect.HitPoint).norm();
			PdfW = CameraVertex->isect.bsdf->Pdf(wo, wi);
			PdfA = ConvertSolidToArea(PdfW, *CameraVertex, *LightVertex);
		}
		a6 = { &LightVertex->PdfPrev, PdfA };
	}
		
	ScopedAssignment<double> a7;
	if (LightVertexMinus) {
		Vec3 wo = (CameraVertex->isect.HitPoint - LightVertex->isect.HitPoint).norm();
		Vec3 wi = (LightVertexMinus->isect.HitPoint - LightVertex->isect.HitPoint).norm();
		double PdfW = LightVertex->isect.bsdf->Pdf(wo, wi);
		double PdfA = ConvertSolidToArea(PdfW, *LightVertex, *LightVertexMinus);
		a7 = { &LightVertexMinus->PdfPrev, PdfA };
	}  


	double sumRi = 0.0;
	auto remap0 = [](double f)->double {return f != 0 ? f : 1; };

	std::vector<double> cc;

	double ri = 1.0;
	for (int i = t - 1; i > 0; --i) {
		ri *= remap0(CameraPath[i].PdfPrev) / remap0(CameraPath[i].PdfFwd);
		if (!CameraPath[i].isect.Delta && !CameraPath[i - 1].isect.Delta)
			sumRi += ri;
	}

	std::vector<double> dd;
	ri = 1.0;
	for (int i = s - 1; i >= 0; --i) {
		ri *= remap0(LightPath[i].PdfPrev) / remap0(LightPath[i].PdfFwd);
		bool deltaLightVertex = i > 0 ? LightPath[i - 1].isect.Delta : false;
		if (!LightPath[i].isect.Delta && !deltaLightVertex)
			sumRi += ri;
	}

	return 1 / (1 + sumRi);
}


double MISWeight2(const std::vector<PathVertex> &LightPath, const std::vector<PathVertex> &CameraPath,
	int s, int t) {
	if (t == 1 && s == 0) std::cout << "t = 1, s = 0" << std::endl;
	if (s + t == 2) return 1.0;
	double ps = 1.0;
	for (int i = 0; i < s; ++i) ps *= LightPath[i].PdfFwd;
	for (int i = t - 1; i >= 0; --i) ps *= CameraPath[i].PdfFwd;

	double p = 0.0;
	for (int i = 0; i < s + t; ++i) {
		double pi = 1.0;
		for (int j = 0; j < s; ++j) {
			if (j < i) pi *= LightPath[j].PdfFwd;
			else pi *= LightPath[j].PdfPrev;
		}
		for (int j = 0; j < t; ++j) {
			int k = s + t - i;
			if (j < k) pi *= CameraPath[j].PdfFwd;
			else pi *= CameraPath[j].PdfPrev;
		}
		p += pi / ps;
	}
	return 1 / p;
}


double PathPdf(const std::vector<PathVertex> &LightPath, const std::vector<PathVertex> &CameraPath,
	int s, int t, bool specialPath) {
	std::vector<PathVertex> FullPath;
	for (int i = 0; i < s; ++i) FullPath.push_back(LightPath[i]);
	for (int i = t - 1; i >= 0; i--) {
		FullPath.push_back(CameraPath[i]);
	}

	auto remap0 = [](double f)->double {return f != 0 ? f : 1; };

	double p = 0.0;
	if (specialPath) {
		double PdfPath = 1.0;
		for (int i = 0; i < s; ++i) PdfPath *= FullPath[i].PdfFwd;
		for (int i = 0; i < t; ++i) {
			int j = s + t - 1 - i;
			PdfPath *= FullPath[j].PdfFwd;
		}
		p += PdfPath;
	}
	else {
		for (int nLightVertex = 0; nLightVertex <= s + t - 1; ++nLightVertex) {
			double PdfPath = 1.0;
			for (int i = 0; i < nLightVertex; ++i) {
				if (i == 0) PdfPath *= FullPath[i].PdfFwd;
				else if (i == 1) PdfPath *= FullPath[i].PdfFwd;
				else {
					const PathVertex &LightVertexi = FullPath[i];
					const PathVertex &LightVertexi_1 = FullPath[i - 1];
					const PathVertex &LightVertexi_2 = FullPath[i - 2];
					Vec3 wo = LightVertexi_2.isect.HitPoint - LightVertexi_1.isect.HitPoint;
					Vec3 wi = LightVertexi.isect.HitPoint - LightVertexi_1.isect.HitPoint;
					double dist = wi.length();
					wo.norm(); wi.norm();
					double PdfW = LightVertexi_1.isect.bsdf->Pdf(wo, wi);
					double CosTheta_i = std::abs(LightVertexi.isect.Normal.dot(wi));
					double PdfA = PdfW * CosTheta_i / dist / dist;
					PdfPath *= PdfA;
				}
			}
			int nCameraVertex = s + t - nLightVertex;
			for (int i = 0; i < nCameraVertex; ++i) {
				int j = s + t - 1 - i;
				if (i == 0) PdfPath *= FullPath[j].PdfFwd;
				else if (i == 1) PdfPath *= FullPath[j].PdfFwd;
				else {
					const PathVertex &CameraVertexi = FullPath[j];
					const PathVertex &CameraVertexi_1 = FullPath[j + 1];
					const PathVertex &CameraVertexi_2 = FullPath[j + 2];
					Vec3 wo = CameraVertexi_2.isect.HitPoint - CameraVertexi_1.isect.HitPoint;
					Vec3 wi = CameraVertexi.isect.HitPoint - CameraVertexi_1.isect.HitPoint;
					double dist = wi.length();
					wo.norm(); wi.norm();
					double PdfW = CameraVertexi_1.isect.bsdf->Pdf(wo, wi);
					double CosTheta_i = std::abs(CameraVertexi.isect.Normal.dot(wi));
					double PdfA = PdfW * CosTheta_i / dist / dist;
					PdfPath *= PdfA;
				}
			}
			
			p += PdfPath;
		}
	}
	return p;
}


double Path_Pdf(const std::vector<PathVertex> &Path, int s, int t) {	
	double p = 1.0;
	for (int i = 0; i < s; ++i) {
		if (i == 0) {
			//double PdfA = Path[0].PdfFwd;
			const Sphere &light = spheres[numSpheres - 1];
			double PdfA = 1.0 / (4 * PI * light.rad * light.rad);
			p *= PdfA;
		}
		else if (i == 1) {
			Vec3 LightToHitPoint = (Path[1].isect.HitPoint - Path[0].isect.HitPoint).norm();
			double CosThetaLight = std::abs(Path[0].isect.Normal.dot(LightToHitPoint));
			double PdfW = CosineHemispherePdf(CosThetaLight);
			double PdfA = ConvertSolidToArea(PdfW, Path[0], Path[1]);
			p *= PdfA;
		}
		else {
			if (Path[i - 1].isect.Delta) p *= 1.0;
			else {
				Vec3 wo = (Path[i - 2].isect.HitPoint - Path[i - 1].isect.HitPoint).norm();
				Vec3 wi = (Path[i].isect.HitPoint - Path[i - 1].isect.HitPoint).norm();
				double PdfW = Path[i - 1].isect.bsdf->Pdf(wo, wi);
				double PdfA = ConvertSolidToArea(PdfW, Path[i - 1], Path[i]);
				p *= PdfA;
			}
		}	
	}
	if (p == 0.0) return 0;
	for (int i = 0; i < t; ++i) {
		int j = s + t - i - 1;
		if (i == 0) {
			//pinhole
			p *= Path[j].camera->PdfPos();
		}
		else if(i == 1){
			Vec3 CameraToHitPoint = (Path[j].isect.HitPoint - Path[j + 1].isect.HitPoint).norm();
			Ray cameraRay(Path[j + 1].isect.HitPoint, CameraToHitPoint);
			double PdfW = Path[j + 1].camera->PdfDir(cameraRay);
			double PdfA = ConvertSolidToArea(PdfW, Path[j + 1], Path[j]);
			p *= PdfA;
		}
		else {
			if (Path[j + 1].isect.Delta) p *= 1.0;
			else {
				Vec3 wo = (Path[j + 2].isect.HitPoint - Path[j + 1].isect.HitPoint).norm();
				Vec3 wi = (Path[j].isect.HitPoint - Path[j + 1].isect.HitPoint).norm();
				double PdfW = Path[j + 1].isect.bsdf->Pdf(wo, wi);
				double PdfA = ConvertSolidToArea(PdfW, Path[j + 1], Path[j]);
				p *= PdfA;
			}
		}
	}
	
	return p;
}

double MISWeight3(std::vector<PathVertex> &LightPath, std::vector<PathVertex> &CameraPath,
	int s, int t, PathVertex &sampled) {
	if (s + t == 2) return 1.0;

	PathVertex *LightVertex = s > 0 ? &LightPath[s - 1] : nullptr,
		*CameraVertex = t > 0 ? &CameraPath[t - 1] : nullptr;

	ScopedAssignment<PathVertex> a1;
	if (s == 1)
		a1 = { LightVertex, sampled };
	else if (t == 1)
		a1 = { CameraVertex, sampled };

	std::vector<PathVertex> FullPath;
	for (int i = 0; i < s; ++i) FullPath.push_back(LightPath[i]);
	for (int i = t - 1; i >= 0; --i) FullPath.push_back(CameraPath[i]);
	double Pdf_s = Path_Pdf(FullPath, s, t);
	double Pdf_all = 0;

	for (int nLightVertices = 0; nLightVertices <= s + t - 1; ++nLightVertices) {
		int nCameraVertices = s + t - nLightVertices;
		if (nLightVertices >= 2 && FullPath[nLightVertices - 1].isect.Delta) continue; 
		if (nLightVertices >= 2 && FullPath[nLightVertices].isect.Delta) continue; 

		Pdf_all += Path_Pdf(FullPath, nLightVertices, nCameraVertices);

	}
	if ((Pdf_s == 0.0) || (Pdf_all == 0.0)) return 0.0;
	else return std::max(std::min(Pdf_s / Pdf_all, 1.0), 0.0);
}

double MISWeight4(const std::vector<PathVertex> &LightPath, const std::vector<PathVertex> &CameraPath,
	int s, int t) {
	double p_i = 1.0, p_all = 0.0;
	for (int i = 0; i < s - 1; ++i) p_i *= LightPath[i].PdfFwd;
	for (int i = 0; i < t - 1; ++i) p_i *= LightPath[i].PdfFwd;
	for (int nLightVertex = 0; nLightVertex <= s + t - 1; ++nLightVertex) {
		double PdfPath = 1.0;
		for (int i = 0; i < nLightVertex; ++i) PdfPath *= LightPath[i].PdfFwd;
		int nCameraVertex = s + t - nLightVertex;
		for (int i = 0; i < nCameraVertex; ++i) PdfPath *= CameraPath[i].PdfFwd;
		p_all += PdfPath;
	}
	return p_i / p_all;
}

Vec3 ConnectBDPT(std::vector<PathVertex> &LightPath, std::vector<PathVertex> &CameraPath,
	int s, int t, Camera &camera, Sampler &sampler, Vec3 *pRaster, bool *inScreen, double *MISRecord) {
	Vec3 L(0, 0, 0);
	PathVertex sampled;
	if (t > 1 && s != 0 && CameraPath[t - 1].isect.IsLight) return Vec3(0, 0, 0);

	if (s == 0) {
		const PathVertex &CameraVertex = CameraPath[t - 1];
		if (CameraVertex.isect.IsLight) L = CameraVertex.Throughput * spheres[numSpheres - 1].e;
	}
	else if (t == 1) {
		
		sampled.camera = &camera;
		//connect to camera
		
		const PathVertex &LightVertex = LightPath[s - 1];
		//pinhole camera
		if (!LightVertex.isect.Delta) {
			
			*inScreen = true;
			Vec3 dirHitPointToCam = camera.o - LightVertex.isect.HitPoint;
			dirHitPointToCam.norm();
			if (camera.d.dot(-1 * dirHitPointToCam) < 0) {
				*inScreen = false;
				L = Vec3(0, 0, 0);
			}
			bool isInScreen;
			*pRaster = camera.WordToScreen(LightVertex.isect.HitPoint, &isInScreen);
			if (!isInScreen) {
				*inScreen = false;
				L = Vec3(0, 0, 0);
			}
			if (!IsConnectible(camera.o, LightVertex.isect.HitPoint)) {
				*inScreen = false;
				L = Vec3(0, 0, 0);
			}
			//if (s == 1) {
			//	return LightVertex.Throughput; //see the light source directly
			//}
			if (*inScreen) {
				double PdfW;
				Vec3 wi;
				Vec3 We = camera.Sample_Wi(LightVertex.isect, &PdfW, &wi);
				Vec3 f = LightVertex.isect.bsdf->f(LightVertex.isect.wo, wi);
				sampled.isect.HitPoint = camera.o;
				sampled.isect.Normal = camera.d;
				sampled.Throughput = We / PdfW;
				sampled.PdfFwd = camera.PdfPos();
				L = We * LightVertex.Throughput * f * wi.dot(LightVertex.isect.Normal) / PdfW;
			}
			
		}
		
	}
	else if (s == 1) {
		//const PathVertex &CameraVertex = CameraPath[t - 1];
		//L = DirectIllumination(CameraVertex.isect, CameraVertex.Throughput, sampler);
		
		const PathVertex &CameraVertex = CameraPath[t - 1];
		if (!CameraVertex.isect.Delta) {
			Sphere &light = spheres[numSpheres - 1];
			Vec3 localZ = (light.p - CameraVertex.isect.HitPoint).norm(), localX, localY;
			CoordinateSystem(localZ, &localX, &localY);

			double SinThetaMax = light.rad / (light.p - CameraVertex.isect.HitPoint).length();
			double CosThetaMax = std::sqrt(1 - SinThetaMax * SinThetaMax);
			Vec3 wi = UniformSampleCone(sampler.Get2D(), CosThetaMax, localX, localY, localZ);
			double PdfW = UniformConePdf(CosThetaMax);

			//calculate the hitpoint and normal
			double CosTheta = wi.dot(localZ);
			double SinTheta = std::sqrt(std::max(0.0, 1 - CosTheta * CosTheta));
			double dc = (light.p - CameraVertex.isect.HitPoint).length();
			double ds = dc * CosTheta - std::sqrt(std::max(0.0, light.rad * light.rad - dc * dc * SinTheta * SinTheta));
			Vec3 HitPoint = CameraVertex.isect.HitPoint + ds * wi;
			Vec3 HitNormal = (HitPoint - light.p).norm();

			Ray shadowRay(CameraVertex.isect.HitPoint, wi);
			double t; int id;
			Intersect isec;
			Vec3 f = CameraVertex.isect.bsdf->f(CameraVertex.isect.wo, wi);
			if (!intersect(shadowRay, t, id, isec) || id != numSpheres - 1) L = Vec3(0.0, 0.0, 0.0);
			else L = CameraVertex.Throughput * f * wi.dot(CameraVertex.isect.Normal) * light.e / PdfW;
			
			sampled.Throughput = light.e / PdfW;
			sampled.PdfFwd = LightPath[0].PdfFwd;
			sampled.isect.HitPoint = HitPoint;
			sampled.isect.Normal = HitNormal;
		}
	}
	else {

		const PathVertex &LightVertex = LightPath[s - 1], &CameraVertex = CameraPath[t - 1];
		//Becareful, if hitpoints on one object, we need to use normal to correct the point position
		if (!LightVertex.isect.Delta && !CameraVertex.isect.Delta 
			&& IsConnectible(LightVertex.isect.HitPoint + eps * LightVertex.isect.Normal
				, CameraVertex.isect.HitPoint + eps * CameraVertex.isect.Normal)) {
			Vec3 cameraTolight = LightVertex.isect.HitPoint - CameraVertex.isect.HitPoint;
			cameraTolight.norm();
			Vec3 fsE = CameraVertex.isect.bsdf->f(CameraVertex.isect.wo, cameraTolight);
			Vec3 fsL = LightVertex.isect.bsdf->f(LightVertex.isect.wo, -1 * cameraTolight);
			double GeometryTerm = G(LightVertex, CameraVertex);
			L = CameraVertex.Throughput * fsE * GeometryTerm * fsL * LightVertex.Throughput;

#ifdef _DEBUG
			/*
			double dis = (LightVertex.isect.HitPoint - CameraVertex.isect.HitPoint).length();
			std::cout << "------------------------------------------------------" << std::endl;
			std::cout << "General connect s: " << s << ", t: " << t 
				<<", s_id: " << LightVertex.isect.id << ", t_id: " << CameraVertex.isect.id
				<< ", LightVertex: " << LightVertex.Throughput
				<< ", CameraVertex: " << CameraVertex.Throughput <<
				", fsL:" << fsL << ", fsE:" << fsE << ", G:" << GeometryTerm
				<< ", L:" << L << ", LightP:" << LightVertex.isect.HitPoint
				<< ", CameraP:" << CameraVertex.isect.HitPoint
				<< ", distance:" << dis * dis << std::endl;*/
#endif
		}
		/*
		else {
			
			if ((int)((*pRaster).x) == 581 && (int)((*pRaster).y) == 493 && t == 2) {
				bool flag;
				Vec3 pos = camera.WordToScreen(LightVertex.isect.HitPoint, &flag);
				if (flag) camera.film->AddVisualPlane(pos);
			}
		}*/
		
	}
	double MIS = (L == Vec3(0.0, 0.0, 0.0) ? 0.0 : MISWeight(LightPath, CameraPath, s, t, sampled));
	L = MIS * L;
	if (MISRecord) *MISRecord = MIS;
	return L;
}

inline int BufferIndex(int s, int t) {
	int above = s + t - 2;
	return s + above * (5 + above) / 2;
}

void BidirectionalPathTracing::Render() {
	
	//Allocate buffers for debug visualization
	const int bufferCount = (1 + maxDepth) * (6 + maxDepth) / 2;
	std::vector<std::unique_ptr<Film> > weightFilms(bufferCount);
	if (visualizeStrategies || visualizeWeight) {
		for (int depth = 0; depth <= maxDepth; ++depth) {
			for (int s = 0; s <= depth + 2; ++s) {
				int t = depth + 2 - s;
				if (t == 0 || (s == 1 && t == 1)) continue;
				std::string filename = "bdpt_d" + std::to_string(depth) + "_s" + std::to_string(s) + "_t" + std::to_string(t) +
					".png";
				weightFilms[BufferIndex(s, t)] = std::unique_ptr<Film>(new Film(800, 600, filename));
			}
		}
	}

	int TotalLightSamples = film->width * film->heigh * spp;
#pragma omp parallel for schedule(dynamic, 1)
	for (int y = 0; y < camera->film->heigh; ++y) {
		fprintf(stderr, "\rRendering %5.2f%%", (1.0 * y * film->width) / film->width / film->heigh * 100.0);
		std::vector<PathVertex> LightPath(maxDepth + 1);
		std::vector<PathVertex> CameraPath(maxDepth + 2);
		for (int x = 0; x < camera->film->width; ++x) {
			//fprintf(stderr, "\rRendering %5.2f%%", (1.0 * y * film->width + x) / film->width / film->heigh * 100.0);
			Vec3 L(0, 0, 0);
			for (int s = 0; s < spp; ++s) {
				double u = sampler->Get1D() - 0.5f;
				double v = sampler->Get1D() - 0.5f;
				double ImageX = x + 0.5f + u;
				double ImageY = y + 0.5f + v;
				double alpha = ImageX / (double)(camera->film->width);
				double beta = ImageY / (double)(camera->film->heigh);
				Vec3 p0 = camera->film->LU + alpha * (camera->film->RU - camera->film->LU);
				Vec3 p1 = camera->film->LL + alpha * (camera->film->RL - camera->film->LL);
				Vec3 p = p0 + beta * (p1 - p0);
				Vec3 d = (p - camera->o).norm();


				Ray cameraRay(camera->o, d);
				//std::vector<PathVertex> LightPath(maxDepth + 1);
				//std::vector<PathVertex> CameraPath(maxDepth + 2);
				int nLightVertex = GenerateLightPath(*sampler, LightPath, maxDepth + 1);
				int nCameraVertex = GenerateCameraPath(*sampler, CameraPath, *camera, cameraRay, maxDepth + 2);
#ifdef _DEBUG
				/*
				
				std::cout << std::endl;
				for (int i = 0; i < nLightVertex; ++i) {
					std::cout << "LightPath[" << i << "], "
						<< "HitPoint: " << LightPath[i].isect.HitPoint
						<< ", Normal: " << LightPath[i].isect.Normal
						<< ", Delta: " << (LightPath[i].isect.Delta ? "true" : "false")
						//<< ", Throughtput: " << LightPath[i].Throughput
						<< ", PdfFwd: " << LightPath[i].PdfFwd 
						<< ", PdfPrev: " << LightPath[i].PdfPrev << std::endl;
				}
				std::cout << std::endl;
				for (int i = 0; i < nCameraVertex; ++i) {
					std::cout << "CameraPath[" << i << "], "
						<< "HitPoint: " << CameraPath[i].isect.HitPoint
						<< ", Normal: " << CameraPath[i].isect.Normal
						<< ", Delta: " << (CameraPath[i].isect.Delta ? "true" : "false")
						//<< ", Throughtput: " << CameraPath[i].Throughput
						<< ", PdfFwd: " << CameraPath[i].PdfFwd
						<< ", PdfPrev: " << CameraPath[i].PdfPrev << std::endl;
				}*/
#endif
				bool inScreen = false;
				for (int t = 1; t <= nCameraVertex; ++t) {
					for (int s = 0; s <= nLightVertex; ++s) {
						//std::cout << "t:" << t << ", s:" << s << std::endl;
						int depth = t + s - 2;
						if (depth < 0 || depth > maxDepth || (s == 1 && t == 1)) continue;
						//if (depth <= 0 || depth > maxDepth ) continue;
						double misRecord;
						Vec3 pRaster = Vec3(x, y);
						Vec3 pFilmNew = Vec3(x, y);
						Vec3 Lpath = ConnectBDPT(LightPath, CameraPath, s, t,
							*camera, *sampler, &pRaster, &inScreen, &misRecord);
#ifdef _DEBUG
						/*
						std::cout << "s:" << s << ",  t:" << t << std::endl;
						for (int i = 0; i < t; ++i) {
							std::cout << "i = " << i << " Throughput=" << CameraPath[i].Throughput;
							std::cout << " IsDelta:" << (CameraPath[i].isect.Delta ? 1 : 0)
								<< " PdfFwd: " << CameraPath[i].PdfFwd << " PdfPrev: " << CameraPath[i].PdfPrev << std::endl;
						}
						std::cout << "mis : " << misRecord << ", L" << Lpath << std::endl<<std::endl;
						*/
#endif
						if (visualizeStrategies || visualizeWeight) {
							if (t == 1 && inScreen) pFilmNew = pRaster;
							Vec3 value;
							if (visualizeStrategies) value = (misRecord == 0.0 ? Vec3(0, 0, 0) : Lpath / misRecord);
							if (visualizeWeight) value = Lpath;
							weightFilms[BufferIndex(s, t)]->AddSplat(pFilmNew, value / spp);
						}

						if (t != 1) {
							L += Lpath / spp;
						}
						else {
							if (inScreen) {
								Lpath = Lpath * film->width * film->heigh / TotalLightSamples;
								//Lpath = Vec3(clamp(Lpath.x), clamp(Lpath.y), clamp(Lpath.z));
								film->AddSplat(pRaster, Lpath);
							}
						}
					}
				}
			}
			//L = Vec3(clamp(L.x), clamp(L.y), clamp(L.z));
			camera->film->AddSample(Vec3(x, y), L);
		}
	}
	camera->film->WriteToImage();
	//camera->film->WriteToVisualImage();
	if (visualizeStrategies || visualizeWeight) {
		for (int i = 0; i < weightFilms.size(); ++i) {
			if (weightFilms[i]) weightFilms[i]->WriteToImage();
		}
	}
}