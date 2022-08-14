#include "BDPTV2.h"
#include "VisibilityTester.h"

namespace bdptV2 {


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
			if (beta == Vec3()) break;
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
	}


	int GenerateCameraSubpath(
		const Scene& scene,
		const Camera& camera,
		MemoryPool &arena,
		Sampler& sampler,
		int maxDepth,
		Vertex* path,
		const Vec2& pFilm)
	{
		if (maxDepth == 0) return 0;
		// Sample initial ray for camera subpath
		CameraSample cameraSample;
		cameraSample.pFilm = pFilm;
		cameraSample.time = sampler.Get1D();
		cameraSample.pLens = sampler.Get2D();
		Ray ray;
		Vec3 beta = camera.GenerateCameraRay(cameraSample, ray);

		// Generate first vertex on camera subpath and start random walk
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
		double time,
		const Distribution1D& lightDistr,
		const std::unordered_map<const Light*, size_t>& lightToIndex,
		Vertex* path)
	{
		if (maxDepth == 0) return 0;
		// Sample initial ray for light subpath
		double lightPdf;
		int lightNum = lightDistr.SampleDiscrete(sampler.Get1D(), &lightPdf);
		const std::shared_ptr<Light>& light = scene.lights[lightNum];
		Ray ray;
		Vec3 nLight;
		double pdfPos, pdfDir;
		Vec3 Le = light->SampleLe(sampler.Get3D(), sampler.Get3D(), time, &ray, &nLight, &pdfPos, &pdfDir);
		if (pdfPos == 0 || pdfDir == 0 || Le == Vec3()) return 0;

		// Generate first vertex on light subpath and start random walk
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
		//*misWeight = L == Vec3(0, 0, 0) ? 0.f : MISWeight(scene, lightVertices, cameraVertices, sampled, s, t, lightDistr, lightToIndex);

		//L = misWeight * L;

		return L;
	}



}

