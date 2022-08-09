#include "BDPT.h"
#include "PathTracing.h"
#include "Scene.h"
#include "LightTracing.h"
#include "Sampling.h"
#include <string>
#include "Utils.h"
#include "Sampling.h"

int GenerateLightPath(const Scene& scene, Sampler &sampler, std::vector<PathVertex> &lightPath, int maxdepth) {
	if (maxdepth == 0) return 0;
	double pdfLight, pdfDir, pdfPos, cosTheta;
	Vec3 dir;
	Intersection lightPoint;
	Light* pLight = scene.SampleOneLight(&pdfLight, sampler.Get1D());
	Vec3 Le = pLight->SampleFromLight(&lightPoint, &dir, &pdfPos, &pdfDir, &cosTheta, sampler);
	Vec3 pos = lightPoint.mPos;
	Vec3 lightDir = dir;

	Ray ray(pos + lightDir * eps, lightDir);
	lightPath[0].mIsect.mPos = pos;
	lightPath[0].mIsect.mNormal = lightPoint.mNormal;
	lightPath[0].mThroughput = Le; // / Pdfdir / Pdfpos * cosTheta;
	lightPath[0].mPdfFwd = pdfPos;
	lightPath[0].mIsect.mIsDelta = false;
	lightPath[0].mIsect.IsLight = true;
	lightPath[0].mIsect.pLight = pLight->GetShape();
	Vec3 throughput = lightPath[0].mThroughput * cosTheta / pdfPos / pdfDir / pdfLight;
	return Trace(scene, ray, throughput, pdfDir, sampler, lightPath, 1, maxdepth - 1);
}

int GenerateCameraPath(const Scene& scene, const Camera& camera, Sampler &sampler, std::vector<PathVertex> &cameraPath, const Ray &cameraRay, int maxdepth) {
	if (maxdepth == 0) return 0;
	Vec3 throughput(1.0, 1.0, 1.0);
	// Throughtput = We * cosTheta / Pdfpos / pdfW = (1, 1, 1)
	cameraPath[0].mIsect.mPos = camera.o;
	cameraPath[0].mIsect.mNormal = camera.d;
	cameraPath[0].mPdfFwd = camera.PdfPos();
	cameraPath[0].mThroughput = throughput;
	double Pdfdir = camera.PdfDir(cameraRay);
	const Camera *cam = &camera;
	cameraPath[0].mpCamera = const_cast<Camera*>(cam);

	return Trace(scene, cameraRay, throughput, Pdfdir, sampler, cameraPath, 1, maxdepth - 1);
}


int Trace(const Scene &scene, const Ray &ray, Vec3 throughput, double pdfFwd, Sampler &sampler, std::vector<PathVertex> &path, int depth, int maxDepth) {
	Ray r = ray;
	int bound = depth;
	double pdfW = pdfFwd;
	while (1) {
		PathVertex &prev = path[bound - 1];
		PathVertex &vertex = path[bound];
		Intersection &isect = path[bound].mIsect;

		if (!scene.Intersect(r, &isect)) break;

		
		path[bound].mThroughput = throughput;
		path[bound].mPdfFwd = ConvertSolidToArea(pdfW, prev, vertex);
 		++bound;
		if (bound >= maxDepth + 1) break;

		Vec3 wo;
		Vec3 f = isect.mpBSDF->Sample_f(-1 * r.d, &wo, &pdfW, sampler.Get3D());
		wo.Normalize();
		throughput = throughput * f * (std::abs(wo.Dot(isect.mNormal))) / pdfW;

		double pdfWPrev = isect.mpBSDF->Pdf(wo, -1 * r.d);
		prev.mPdfPrev = ConvertSolidToArea(pdfWPrev, vertex, prev);
		
		if (isect.mIsDelta) {
			pdfW = pdfWPrev = 0;
		}

		r.o = isect.mPos;// +wo * eps;
		r.d = wo;
	}
	return bound;
}


double ConvertSolidToArea(double pdfW, const PathVertex &vertex, const PathVertex &nextVertex) {
	Vec3 dir = nextVertex.mIsect.mPos - vertex.mIsect.mPos;
	double dist = dir.Length();
	double dist2 = dist * dist;
	if (dist2 == 0) return 0;
	dir.Normalize();
	double cosTheta = std::abs(dir.Dot(nextVertex.mIsect.mNormal));
	return pdfW * cosTheta / dist2;
}

bool IsConnectable(const Scene& scene, const Vec3& pointA, const Vec3& pointB) {
	Vec3 dir = pointA - pointB;
	double dist = dir.Length();
	Ray ray(pointB, dir.Norm(), 0.0, dist - eps);  //Be careful ! Float error will cause the incorrect intersection
	
	if (scene.Intersect(ray)) return false;
	return true;
}

double G(const PathVertex &vertexA, const PathVertex &vertexB) {
	Vec3 dirAtoB = vertexB.mIsect.mPos - vertexA.mIsect.mPos;
	double dist = dirAtoB.Length();
	dirAtoB.Normalize();
	double cosThetaA = std::abs(dirAtoB.Dot(vertexA.mIsect.mNormal));
	double cosThetaB = std::abs(dirAtoB.Dot(vertexB.mIsect.mNormal));
	double g = cosThetaA * cosThetaB / dist / dist;
	return g;
}


double MISWeight(std::vector<PathVertex>& lightPath, std::vector<PathVertex>& cameraPath, int s, int t, PathVertex& sampled) {
	if (s + t == 2) return 1;

	PathVertex* lightVertex = s > 0 ? &lightPath[s - 1] : nullptr,
		* cameraVertex = t > 0 ? &cameraPath[t - 1] : nullptr,
		* lightVertexMinus = s > 1 ? &lightPath[s - 2] : nullptr,
		* cameraVertexMinus = t > 1 ? &cameraPath[t - 2] : nullptr;

	ScopedAssignment<PathVertex> a1;
	if (s == 1)
		a1 = { lightVertex, sampled };
	else if (t == 1)
		a1 = { cameraVertex, sampled };


	ScopedAssignment<bool> a2, a3;
	if (lightVertex) a2 = { &lightVertex->mIsect.mIsDelta, false };
	if (cameraVertex) a3 = { &cameraVertex->mIsect.mIsDelta, false };

	ScopedAssignment<double> a4;
	if (cameraVertex) {
		if (s > 0) {
			double pdfW, pdfA;
			if (lightVertexMinus == nullptr) {
				Vec3 lightToHitPoint = cameraVertex->mIsect.mPos - lightVertex->mIsect.mPos;
				lightToHitPoint.Norm();
				double cosTheta = lightVertex->mIsect.mNormal.Dot(lightToHitPoint);
				pdfW = CosineHemispherePdf(cosTheta);
				pdfA = ConvertSolidToArea(pdfW, *lightVertex, *cameraVertex);
			}
			else {
				Vec3 wo = (lightVertexMinus->mIsect.mPos - lightVertex->mIsect.mPos).Norm();
				Vec3 wi = (cameraVertex->mIsect.mPos - lightVertex->mIsect.mPos).Norm();
				pdfW = lightVertex->mIsect.mpBSDF->Pdf(wo, wi);
				pdfA = ConvertSolidToArea(pdfW, *lightVertex, *cameraVertex);
			}
			a4 = { &cameraVertex->mPdfPrev, pdfA };
		}
		else {
			double pdfA = 1.0 / cameraVertex->mIsect.pLight->Area();
			a4 = { &cameraVertex->mPdfPrev, pdfA };
		}
	}

	ScopedAssignment<double> a5;
	if (cameraVertexMinus) {
		if (s > 0) {
			Vec3 wo = (lightVertex->mIsect.mPos - cameraVertex->mIsect.mPos).Norm();
			Vec3 wi = (cameraVertexMinus->mIsect.mPos - cameraVertex->mIsect.mPos).Norm();
			double pdfW = cameraVertex->mIsect.mpBSDF->Pdf(wo, wi);
			double pdfA = ConvertSolidToArea(pdfW, *cameraVertex, *cameraVertexMinus);
			a5 = { &cameraVertexMinus->mPdfPrev, pdfA };
		}
		else {
			Vec3 hitPointToLight = cameraVertex->mIsect.mPos - cameraVertexMinus->mIsect.mPos;
			double dist = hitPointToLight.Length();
			hitPointToLight.Normalize();
			Vec3 lightToHitPoint = -1 * hitPointToLight;
			Vec3 lightNormal = cameraVertex->mIsect.mNormal;
			double cosThetaLight = std::abs(lightNormal.Dot(lightToHitPoint));
			double pdfW = CosineHemispherePdf(cosThetaLight);
			double pdfA = pdfW * (std::abs(cameraVertexMinus->mIsect.mNormal.Dot(hitPointToLight))) / dist / dist;
			a5 = { &cameraVertexMinus->mPdfPrev, pdfA };
		}

	}

	// Update reverse density of vertices $\pq{}_{s-1}$ and $\pq{}_{s-2}$
	ScopedAssignment<double> a6;
	if (lightVertex) {
		double pdfW, pdfA;
		if (cameraVertexMinus == nullptr) {
			Vec3 cameraToHitPoint = lightVertex->mIsect.mPos - cameraVertex->mIsect.mPos;
			cameraToHitPoint.Normalize();
			double cosTheta = std::abs(cameraVertex->mIsect.mNormal.Dot(cameraToHitPoint));
			Ray ray(cameraVertex->mIsect.mPos, cameraToHitPoint);
			pdfW = cameraVertex->mpCamera->PdfDir(ray);
			pdfA = ConvertSolidToArea(pdfW, *cameraVertex, *lightVertex);
		}
		else {
			Vec3 wo = (cameraVertexMinus->mIsect.mPos - cameraVertex->mIsect.mPos).Norm();
			Vec3 wi = (lightVertex->mIsect.mPos - cameraVertex->mIsect.mPos).Norm();
			pdfW = cameraVertex->mIsect.mpBSDF->Pdf(wo, wi);
			pdfA = ConvertSolidToArea(pdfW, *cameraVertex, *lightVertex);
		}
		a6 = { &lightVertex->mPdfPrev, pdfA };
	}

	ScopedAssignment<double> a7;
	if (lightVertexMinus) {
		Vec3 wo = (cameraVertex->mIsect.mPos - lightVertex->mIsect.mPos).Norm();
		Vec3 wi = (lightVertexMinus->mIsect.mPos - lightVertex->mIsect.mPos).Norm();
		double pdfW = lightVertex->mIsect.mpBSDF->Pdf(wo, wi);
		double pdfA = ConvertSolidToArea(pdfW, *lightVertex, *lightVertexMinus);
		a7 = { &lightVertexMinus->mPdfPrev, pdfA };
	}


	double sumRi = 0.0;
	auto remap0 = [](double f)->double {return f != 0 ? f : 1; };

	std::vector<double> cc;

	double ri = 1.0;
	for (int i = t - 1; i > 0; --i) {
		ri *= remap0(cameraPath[i].mPdfPrev) / remap0(cameraPath[i].mPdfFwd);
		if (!cameraPath[i].mIsect.mIsDelta && !cameraPath[i - 1].mIsect.mIsDelta)
			sumRi += ri;
	}

	std::vector<double> dd;
	ri = 1.0;
	for (int i = s - 1; i >= 0; --i) {
		ri *= remap0(lightPath[i].mPdfPrev) / remap0(lightPath[i].mPdfFwd);
		bool deltaLightVertex = i > 0 ? lightPath[i - 1].mIsect.mIsDelta : false;
		if (!lightPath[i].mIsect.mIsDelta && !deltaLightVertex)
			sumRi += ri;
	}

	return 1 / (1 + sumRi);
}

double Path_Pdf(const std::vector<PathVertex>& path, int s, int t) {
	double p = 1.0;
	for (int i = 0; i < s; ++i) {
		if (i == 0) {

			double pdfA = 1.0 / path[0].mIsect.pLight->Area();
			p *= pdfA;
		}
		else if (i == 1) {
			Vec3 lightToHitPoint = (path[1].mIsect.mPos - path[0].mIsect.mPos).Norm();
			double cosThetaLight = std::abs(path[0].mIsect.mNormal.Dot(lightToHitPoint));
			double pdfW = CosineHemispherePdf(cosThetaLight);
			double pdfA = ConvertSolidToArea(pdfW, path[0], path[1]);
			p *= pdfA;
		}
		else {
			if (path[i - 1].mIsect.mIsDelta) p *= 1.0;
			else {
				Vec3 wo = (path[i - 2].mIsect.mPos - path[i - 1].mIsect.mPos).Norm();
				Vec3 wi = (path[i].mIsect.mPos - path[i - 1].mIsect.mPos).Norm();
				double pdfW = path[i - 1].mIsect.mpBSDF->Pdf(wo, wi);
				double pdfA = ConvertSolidToArea(pdfW, path[i - 1], path[i]);
				p *= pdfA;
			}
		}
	}
	if (p == 0.0) return 0;
	for (int i = 0; i < t; ++i) {
		int j = s + t - i - 1;
		if (i == 0) {
			//pinhole
			p *= path[j].mpCamera->PdfPos();
		}
		else if (i == 1) {
			Vec3 CameraToHitPoint = (path[j].mIsect.mPos - path[j + 1].mIsect.mPos).Norm();
			Ray cameraRay(path[j + 1].mIsect.mPos, CameraToHitPoint);
			double pdfW = path[j + 1].mpCamera->PdfDir(cameraRay);
			double pdfA = ConvertSolidToArea(pdfW, path[j + 1], path[j]);
			p *= pdfA;
		}
		else {
			if (path[j + 1].mIsect.mIsDelta) p *= 1.0;
			else {
				Vec3 wo = (path[j + 2].mIsect.mPos - path[j + 1].mIsect.mPos).Norm();
				Vec3 wi = (path[j].mIsect.mPos - path[j + 1].mIsect.mPos).Norm();
				double pdfW = path[j + 1].mIsect.mpBSDF->Pdf(wo, wi);
				double pdfA = ConvertSolidToArea(pdfW, path[j + 1], path[j]);
				p *= pdfA;
			}
		}
	}
	return p;
}

double MISWeight2(std::vector<PathVertex>& lightPath, std::vector<PathVertex>& cameraPath, int s, int t, PathVertex& sampled) {
	if (s + t == 2) return 1.0;

	PathVertex* lightVertex = s > 0 ? &lightPath[s - 1] : nullptr,
		* cameraVertex = t > 0 ? &cameraPath[t - 1] : nullptr;

	//ScopedAssignment<PathVertex> a1;
	//if (s == 1)
	//	a1 = { lightVertex, sampled };
	//else if (t == 1)
	//	a1 = { cameraVertex, sampled };

	std::vector<PathVertex> fullPath;
	for (int i = 0; i < s; ++i) fullPath.push_back(lightPath[i]);
	for (int i = t - 1; i >= 0; --i) fullPath.push_back(cameraPath[i]);
	double Pdf_s = Path_Pdf(fullPath, s, t);
	double Pdf_all = 0;

	for (int nLightVertices = 0; nLightVertices <= s + t - 1; ++nLightVertices) {
		int nCameraVertices = s + t - nLightVertices;
		if (nLightVertices >= 2 && fullPath[nLightVertices - 1].mIsect.mIsDelta) continue;
		if (nLightVertices >= 2 && fullPath[nLightVertices].mIsect.mIsDelta) continue;

		Pdf_all += Path_Pdf(fullPath, nLightVertices, nCameraVertices);

	}
	if ((Pdf_s == 0.0) || (Pdf_all == 0.0)) return 0.0;
	else return std::max(std::min(Pdf_s / Pdf_all, 1.0), 0.0);
}


Vec3 ConnectBDPT(const Scene& scene, const Camera& camera, Sampler& sampler, std::vector<PathVertex>& lightPath, std::vector<PathVertex>& cameraPath, int s, int t, Vec3* pRaster, bool* inScreen, double* MISRecord) {
	Vec3 L(0, 0, 0);
	PathVertex sampled;
	if (t > 1 && s != 0 && cameraPath[t - 1].mIsect.IsLight) return Vec3(0, 0, 0);

	if (s == 0) {
		const PathVertex& cameraVertex = cameraPath[t - 1];
		if (cameraVertex.mIsect.IsLight && cameraVertex.mIsect.pLight->GetNormal(cameraVertex.mIsect.mPos).Dot(cameraVertex.mIsect.mOutDir) > 0)
		{
			L = cameraVertex.mThroughput * cameraVertex.mIsect.pLight->Emission();
		}
	}
	else if (t == 1) {

		sampled.mpCamera = const_cast<Camera*>(&camera);
		//connect to camera

		const PathVertex& lightVertex = lightPath[s - 1];
		//pinhole camera
		if (!lightVertex.mIsect.mIsDelta) {

			*inScreen = true;
			Vec3 dirHitPointToCam = camera.o - lightVertex.mIsect.mPos;
			dirHitPointToCam.Normalize();
			if (camera.d.Dot(-1 * dirHitPointToCam) < 0) {
				*inScreen = false;
				L = Vec3(0, 0, 0);
			}
			bool isInScreen;
			*pRaster = camera.WordToScreen(lightVertex.mIsect.mPos, &isInScreen);
			
			if (!isInScreen) {
				*inScreen = false;
				L = Vec3(0, 0, 0);
			}
			if (!IsConnectable(scene, camera.o, lightVertex.mIsect.mPos)) {
				*inScreen = false;
				L = Vec3(0, 0, 0);
			}
			//if (s == 1) {
			//	return lightVertex.Throughput; //see the light source directly
			//}
			if (*inScreen) {
				double pdfW;
				Vec3 wi;
				Vec3 We = camera.Sample_Wi(lightVertex.mIsect, &pdfW, &wi);
				Vec3 f = lightVertex.mIsect.mpBSDF->f(lightVertex.mIsect.mOutDir, wi);
				sampled.mIsect.mPos = camera.o;
				sampled.mIsect.mNormal = camera.d;
				sampled.mThroughput = We / pdfW;
				sampled.mPdfFwd = camera.PdfPos();
				L = We * lightVertex.mThroughput * f * wi.Dot(lightVertex.mIsect.mNormal) / pdfW;
			}

		}

	}
	else if (s == 1) {
		const PathVertex& cameraVertex = cameraPath[t - 1];
		if (!cameraVertex.mIsect.mIsDelta) {

			double pdfLight;
			PathVertex sampledVertex;
			Light* pLight = scene.SampleOneLight(&pdfLight, sampler.Get1D());
			L = pLight->DirectIllumination(scene, sampler, cameraVertex.mIsect, cameraVertex.mThroughput, &sampledVertex) / pdfLight;

			sampled.mThroughput = sampledVertex.mThroughput / pdfLight;
			sampled.mPdfFwd = lightPath[0].mPdfFwd;
			sampled.mIsect.mPos = sampledVertex.mIsect.mPos;
			sampled.mIsect.mNormal = sampledVertex.mIsect.mNormal;

		}
	}
	else {

		const PathVertex& lightVertex = lightPath[s - 1], & cameraVertex = cameraPath[t - 1];
		//Be careful, if hit points on one object, we need to use normal to correct the point position
		if (!lightVertex.mIsect.mIsDelta && !cameraVertex.mIsect.mIsDelta
			&& IsConnectable(scene, lightVertex.mIsect.mPos + eps * lightVertex.mIsect.mNormal
				, cameraVertex.mIsect.mPos + eps * cameraVertex.mIsect.mNormal))
		{
			Vec3 cameraTolight = lightVertex.mIsect.mPos - cameraVertex.mIsect.mPos;
			cameraTolight.Norm();
			Vec3 fsE = cameraVertex.mIsect.mpBSDF->f(cameraVertex.mIsect.mOutDir, cameraTolight);
			Vec3 fsL = lightVertex.mIsect.mpBSDF->f(lightVertex.mIsect.mOutDir, -1 * cameraTolight);
			double GeometryTerm = G(lightVertex, cameraVertex);
			L = cameraVertex.mThroughput * fsE * GeometryTerm * fsL * lightVertex.mThroughput;
		}
	}
	//double MIS = (L == Vec3(0.0, 0.0, 0.0) ? 0.0 : MISWeight(lightPath, cameraPath, s, t, sampled));
	double MIS2 = (L == Vec3(0.0, 0.0, 0.0) ? 0.0 : MISWeight2(lightPath, cameraPath, s, t, sampled));
	//if (std::abs(MIS - MIS2) > 1e-5) {
	//	std::cout << "MIS: " << MIS << " MIS2: " << MIS2 << std::endl;
	//}
	//std::cout << "MIS: " << MIS << " MIS2: " << MIS2 << std::endl;
	L = MIS2 * L;
	if (MISRecord) *MISRecord = MIS2;
	return L;
}

inline int BufferIndex(int s, int t) {
	int above = s + t - 2;
	return s + above * (5 + above) / 2;
}

void BidirectionalPathTracing::Render(const Scene& scene, const Camera& camera) {
	
	//Allocate buffers for debug visualization
	const int bufferCount = (1 + mMaxDepth) * (6 + mMaxDepth) / 2;
	std::vector<std::unique_ptr<Film> > weightFilms(bufferCount);
	if (mVisualizeStrategies || mVisualizeWeight) {
		for (int depth = 0; depth <= mMaxDepth; ++depth) {
			for (int s = 0; s <= depth + 2; ++s) {
				int t = depth + 2 - s;
				if (t == 0 || (s == 1 && t == 1)) continue;
				std::string filename = "bdpt_d" + std::to_string(depth) + "_s" + std::to_string(s) + "_t" + std::to_string(t) + ".png";
				weightFilms[BufferIndex(s, t)] = std::unique_ptr<Film>(new Film(800, 600, filename));
			}
		}
	}

	Film* film = camera.GetFilm();
	int64_t resX = film->resX;
	int64_t resY = film->resY;
	int64_t totalLightSamples = resX * resY * mSpp;
#pragma omp parallel for schedule(dynamic, 1)
	for (int y = 0; y < resY; ++y) {
		fprintf(stderr, "\rRendering %5.2f%%", (1.0 * y * resX) / resX / resY * 100.0);
		std::vector<PathVertex> lightPath(mMaxDepth + 1);
		std::vector<PathVertex> cameraPath(mMaxDepth + 2);
		for (int x = 0; x < resX; ++x) {
			Vec3 L(0, 0, 0);
			for (int spp = 0; spp < mSpp; ++spp) {
				Ray cameraRay = camera.GenerateRay(x, y, mpSampler->Get2D(), 0);
				int nLightVertex = GenerateLightPath(scene, *mpSampler, lightPath, mMaxDepth + 1);
				int nCameraVertex = GenerateCameraPath(scene, camera, *mpSampler, cameraPath, cameraRay, mMaxDepth + 2);

				bool inScreen = false;
				for (int t = 1; t <= nCameraVertex; ++t) {
					for (int s = 0; s <= nLightVertex; ++s) {
						//std::cout << "t:" << t << ", s:" << s << std::endl;
						int depth = t + s - 2;
						if (depth < 0 || depth > mMaxDepth || (s == 1 && t == 1)) continue;
						//if (depth <= 0 || depth > maxDepth ) continue;
						double misRecord;
						Vec3 pRaster = Vec3(x, y, 0);
						Vec3 pFilmNew = Vec3(x, y, 0);
						Vec3 Lpath = ConnectBDPT(scene, camera, *mpSampler, lightPath, cameraPath, s, t, &pRaster, &inScreen, &misRecord);

						if (mVisualizeStrategies || mVisualizeWeight) {
							if (t == 1 && inScreen) pFilmNew = pRaster;
							Vec3 value;
							if (mVisualizeStrategies) value = (misRecord == 0.0 ? Vec3(0, 0, 0) : Lpath / misRecord);
							if (mVisualizeWeight) value = Lpath;
							weightFilms[BufferIndex(s, t)]->AddSplat(pFilmNew, value / mSpp);
						}

						if (t != 1) {
							L += Lpath / mSpp;
						}
						else {
							if (inScreen) {
								Lpath = Lpath * film->resX * film->resY / totalLightSamples;

								film->AddSplat(pRaster, Lpath);
							}
						}
					}
				}
			}
			film->AddSample(Vec3(x, y, 0), L);
		}
	}
	film->WriteToImage();
	if (mVisualizeStrategies || mVisualizeWeight) {
		for (int i = 0; i < weightFilms.size(); ++i) {
			if (weightFilms[i]) weightFilms[i]->WriteToImage();
		}
	}
}