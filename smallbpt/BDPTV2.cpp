#include "BDPTV2.h"
#include "VisibilityTester.h"
#include "ScopedAssignment.h"

double CorrectShadingNormal(const SurfaceIntersection& isect, const Vec3& wo, const Vec3& wi, TransportMode mode)
{
	if (mode == TransportMode::Importance) {
		double num = Dot(wo, isect.mNormal) * fabs(Dot(wi, isect.mNormal));
		double denom = fabs(Dot(wo, isect.mNormal)) * fabs(Dot(wi, isect.mNormal));
		// wi is occasionally perpendicular to isect.shading.n; this is
		// fine, but we don't want to return an infinite or NaN value in
		// that case.
		if (denom == 0) return 0;
		return num / denom;
	}
	else
		return 1;
}

int RandomWalk(
	const Scene& scene,
	Ray ray,
	Sampler& sampler,
	MemoryPool &arena,
	Vec3 beta,
	double pdfDir,
	int maxDepth,
	TransportMode mode,
	Vertex* path) 
{
	if (maxDepth == 0) return 0;
	int bounces = 0;
	// Declare variables for forward and reverse probability densities
	double pdfFwd = pdfDir, pdfRev = 0;
	while (true) {
		// Attempt to create the next subpath vertex in _path_

		// Trace a ray and sample the medium, if any
		SurfaceIntersection isect;
		bool foundIntersection = scene.Intersect(ray, &isect);
		
		Vertex &vertex = path[bounces], &prev = path[bounces - 1];

		// Handle surface interaction for path generation
		if (!foundIntersection) {
			// Capture escaped rays when tracing from the camera
			if (mode == TransportMode::Radiance) {
				vertex = Vertex::CreateLight(EndpointIntersection(ray), beta, pdfFwd);
				++bounces;
			}
			break;
		}


		// Compute scattering functions for _mode_ and skip over medium
		// boundaries
		if (!isect.mpBSDF) {
			ray = isect.SpawnRay(ray.d);
			continue;
		}

		// Initialize _vertex_ with surface intersection information
		vertex = Vertex::CreateSurface(isect, beta, pdfFwd, prev);
		if (++bounces >= maxDepth) break;

		// Sample BSDF at current vertex and compute reverse probability
		Vec3 wi, wo = isect.mOutDir;
		BxDFType type;
		Vec3 f = isect.mpBSDF->Sample_f(wo, &wi, &pdfFwd, sampler.Get3D(), &type);

		if (f == Vec3() || pdfFwd == 0.f) break;

		beta = beta * f * fabs(Dot(wi, isect.mNormal)) / pdfFwd;
		pdfRev = isect.mpBSDF->Pdf(wi, wo);
		if ((type & BSDF_SPECULAR) || (type & BSDF_TRANSMISSION)) {
			vertex.delta = true;
			pdfRev = pdfFwd = 0;
		}
		beta = beta * CorrectShadingNormal(isect, wo, wi, mode);
		ray = isect.SpawnRay(wi);
		// Compute reverse area density at preceding vertex
		prev.pdfRev = vertex.ConvertDensity(pdfRev, prev);
	}
	return bounces;
}

int GenerateCameraSubpath(
	const Scene& scene, 
	const Camera& camera,
	int x, int y,
	Sampler& sampler,
	MemoryPool& arena, 
	int maxDepth,
	const Vec2& pFilm,
	Vertex* path)
{
	if (maxDepth == 0) return 0;
	// Sample initial ray for camera sub path
	CameraSample cameraSample;
	cameraSample.pFilm = pFilm;
	Ray ray;
	Vec3 beta = camera.GenerateCameraRay(x, y, cameraSample, ray);
	// Generate first vertex on camera sub path and start random walk
	double pdfPos, pdfDir;
	path[0] = Vertex::CreateCamera(&camera, ray, beta);
	camera.PdfWe(ray, &pdfPos, &pdfDir);
	return RandomWalk(scene, ray, sampler, arena, beta, pdfDir, maxDepth - 1, TransportMode::Radiance, path + 1) + 1;
}

int GenerateLightSubpath(
	const Scene& scene,
	Sampler& sampler,
	MemoryPool& arena,
	int maxDepth,
	const std::unordered_map<const Light*, size_t>& lightToIndex,
	Vertex* path)
{
	if (maxDepth == 0) return 0;
	// Sample initial ray for light sub path
	double lightPdf = lightPdf = 1.0 / scene.lights.size();
	int lightNum = std::min((int)(scene.lights.size() * sampler.Get1D()), 1);
	const std::shared_ptr<Light>& light = scene.lights[lightNum];
	Ray ray;
	Vec3 nLight;
	double pdfPos, pdfDir;
	Vec3 Le = light->SampleLe(sampler.Get3D(), sampler.Get3D(), &ray, &nLight, &pdfPos, &pdfDir);
	if (pdfPos == 0 || pdfDir == 0 || Le == Vec3()) return 0;

	// Generate first vertex on light sub path and start random walk
	path[0] =
		Vertex::CreateLight(light.get(), ray, nLight, Le, pdfPos * lightPdf);
	Vec3 beta = Le * std::fabs(Dot(nLight, ray.d) / (lightPdf * pdfPos * pdfDir));

	int nVertices = RandomWalk(scene, ray, sampler, arena, beta, pdfDir, maxDepth - 1, TransportMode::Importance, path + 1);

	return nVertices + 1;
}

Vec3 G(const Vertex& v0, const Vertex& v1) {
	Vec3 d = v0.p() - v1.p();
	double g = 1 / d.Dot(d);
	d = d * std::sqrt(g);
	if (v0.IsOnSurface()) g *= fabs(Dot(v0.ns(), d));
	if (v1.IsOnSurface()) g *= fabs(Dot(v1.ns(), d));
	//VisibilityTester vis(v0.GetIntersection(), v1.GetIntersection());
	return g;
}

double MISWeight(const Scene& scene, Vertex* lightVertices,
	Vertex* cameraVertices, Vertex& sampled, int s, int t,
	const Distribution1D& lightPdf,
	const std::unordered_map<const Light*, size_t>& lightToIndex) {
	if (s + t == 2) return 1;
	double sumRi = 0;
	// Define helper function _remap0_ that deals with Dirac delta functions
	auto remap0 = [](double f) -> double { return f != 0 ? f : 1; };

	// Temporarily update vertex properties for current strategy

	// Look up connection vertices and their predecessors
	Vertex* qs = s > 0 ? &lightVertices[s - 1] : nullptr,
		* pt = t > 0 ? &cameraVertices[t - 1] : nullptr,
		* qsMinus = s > 1 ? &lightVertices[s - 2] : nullptr,
		* ptMinus = t > 1 ? &cameraVertices[t - 2] : nullptr;

	// Update sampled vertex for $s=1$ or $t=1$ strategy
	ScopedAssignment<Vertex> a1;
	if (s == 1)
		a1 = { qs, sampled };
	else if (t == 1)
		a1 = { pt, sampled };

	// Mark connection vertices as non-degenerate
	ScopedAssignment<bool> a2, a3;
	if (pt) a2 = { &pt->delta, false };
	if (qs) a3 = { &qs->delta, false };

	// Update reverse density of vertex $\pt{}_{t-1}$
	ScopedAssignment<double> a4;
	if (pt)
		a4 = { &pt->pdfRev, s > 0 ? qs->Pdf(scene, qsMinus, *pt)
									: pt->PdfLightOrigin(scene, *ptMinus, lightPdf,
														lightToIndex) };

	// Update reverse density of vertex $\pt{}_{t-2}$
	ScopedAssignment<double> a5;
	if (ptMinus)
		a5 = { &ptMinus->pdfRev, s > 0 ? pt->Pdf(scene, qs, *ptMinus)
										: pt->PdfLight(scene, *ptMinus) };

	// Update reverse density of vertices $\pq{}_{s-1}$ and $\pq{}_{s-2}$
	ScopedAssignment<double> a6;
	if (qs) a6 = { &qs->pdfRev, pt->Pdf(scene, ptMinus, *qs) };
	ScopedAssignment<double> a7;
	if (qsMinus) a7 = { &qsMinus->pdfRev, qs->Pdf(scene, pt, *qsMinus) };

	// Consider hypothetical connection strategies along the camera subpath
	double ri = 1;
	for (int i = t - 1; i > 0; --i) {
		ri *=
			remap0(cameraVertices[i].pdfRev) / remap0(cameraVertices[i].pdfFwd);
		if (!cameraVertices[i].delta && !cameraVertices[i - 1].delta)
			sumRi += ri;
	}

	// Consider hypothetical connection strategies along the light subpath
	ri = 1;
	for (int i = s - 1; i >= 0; --i) {
		ri *= remap0(lightVertices[i].pdfRev) / remap0(lightVertices[i].pdfFwd);
		bool deltaLightvertex = i > 0 ? lightVertices[i - 1].delta
			: lightVertices[0].IsDeltaLight();
		if (!lightVertices[i].delta && !deltaLightvertex) sumRi += ri;
	}
	return 1 / (1 + sumRi);
}

Vec3 ConnectBDPT(
	const Scene& scene,
	const Camera& camera,
	Sampler& sampler,
	Vertex* lightVertices,
	Vertex* cameraVertices,
	int s,
	int t,
	const Distribution1D& lightDistr,
	const std::unordered_map<const Light*, size_t>& lightToIndex,
	Vec2* pRaster,
	double* misWeight /*= nullptr*/)
{
	Vec3 L;
	// Ignore invalid connections related to infinite area lights
	if (t > 1 && s != 0 && cameraVertices[t - 1].type == VertexType::Light)
		return Vec3(0.f);

	// Perform connection and write contribution to _L_
	Vertex sampled;
	if (s == 0) {
		// Interpret the camera subpath as a complete path
		const Vertex& pt = cameraVertices[t - 1];
		if (pt.IsLight()) L = pt.Le(scene, cameraVertices[t - 2]) * pt.beta;
	}
	else if (t == 1) {
		// Sample a point on the camera and connect it to the light subpath
		const Vertex& qs = lightVertices[s - 1];
		if (qs.IsConnectible()) {
			VisibilityTester vis;
			Vec3 wi;
			double pdfW;
			Vec3 Wi = camera.Sample_Wi(qs.GetIntersection(), &pdfW, &wi, sampler.Get3D());
			if (pdfW > 0 && Wi != Vec3()) {
				// Initialize dynamically sampled vertex and _L_ for $t=1$ case
				sampled = Vertex::CreateCamera(&camera, vis.P1(), Wi / pdfW);
				L = qs.beta * qs.f(sampled, TransportMode::Importance) * sampled.beta;
				if (qs.IsOnSurface()) {
					L = L * fabs(Dot(wi, qs.ns()));
				}

			}
		}
	}
	else if (s == 1) {
		// Sample a point on a light and connect it to the camera subpath
		const Vertex& pt = cameraVertices[t - 1];
		if (pt.IsConnectible()) {
			double lightPdf;
			VisibilityTester vis;
			Vec3 wi;
			double pdf;
			int lightNum = lightDistr.SampleDiscrete(sampler.Get1D(), &lightPdf);

			const std::shared_ptr<Light>& light = scene.lights[lightNum];

			Vec3 lightWeight = light->SampleLi(pt.GetIntersection(), sampler.Get3D(), &wi, &pdf, &vis);

			if (pdf > 0 && lightWeight != Vec3()) {
				EndpointIntersection ei(vis.P1(), light.get());

				sampled = Vertex::CreateLight(ei, lightWeight / (pdf * lightPdf), 0);

				sampled.pdfFwd = sampled.PdfLightOrigin(scene, pt, lightDistr, lightToIndex);

				L = pt.beta * pt.f(sampled, TransportMode::Radiance) * sampled.beta;

				if (pt.IsOnSurface()) L = L * fabs(Dot(wi, pt.ns()));
				// Only check visibility if the path would carry radiance.
				//if (!L.IsBlack()) L *= vis.Tr(scene, sampler);
			}
		}
	}
	else {
		// Handle all other bidirectional connection cases
		const Vertex& qs = lightVertices[s - 1], & pt = cameraVertices[t - 1];
		if (qs.IsConnectible() && pt.IsConnectible()) {
			L = qs.beta * qs.f(pt, TransportMode::Importance) * pt.f(qs, TransportMode::Radiance) * pt.beta;

			if (L != Vec3()) L = L * G(qs, pt);
		}
	}

	// Compute MIS weight for connection strategy
	*misWeight = L == Vec3(0, 0, 0) ? 0.f : MISWeight(scene, lightVertices, cameraVertices, sampled, s, t, lightDistr, lightToIndex);

	L = *misWeight * L;

	return L;
}



void BDPTIntegrator::Render(const Scene& scene, const Camera& camera)
{
	std::unordered_map<const Light*, size_t> lightToIndex;
	for (size_t i = 0; i < scene.lights.size(); ++i)
		lightToIndex[scene.lights[i].get()] = i;

	Film* film = camera.GetFilm();
	int64 resX = film->resX;
	int64 resY = film->resY;
	int64 totalLightSamples = resX * resY * spp;

	for (int64 y = 0; y < resY; y++) {
		MemoryPool arena;
		for (int64 x = 0; x < resX; x++) {
			Vertex* cameraVertices = arena.Alloc<Vertex>(maxDepth + 2);
			Vertex* lightVertices = arena.Alloc<Vertex>(maxDepth + 1);
			Vec3 L(0, 0, 0);
			for (int s = 0; s < spp; s++) {
				Vec2 pFilm = Vec2(x, y) + sampler->Get2D();

				Ray cameraRay = camera.GenerateRay(x, y, sampler->Get2D(), 0);
				Vertex* cameraVertices = arena.Alloc<Vertex>(maxDepth + 2);
				Vertex* lightVertices = arena.Alloc<Vertex>(maxDepth + 1);

				int nCamera = GenerateCameraSubpath(scene, camera, x, y, *sampler, arena, maxDepth + 2, pFilm, cameraVertices);
				int nLight = GenerateLightSubpath(scene, *sampler, arena, maxDepth + 1, lightToIndex, lightVertices);

				std::cout << "\nnCamera:" << nCamera << " nLight: " << nLight << std::endl;
			}
		}
	}
}
