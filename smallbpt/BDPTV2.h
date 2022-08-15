#pragma once
#include "Intersection.h"
#include "Camera.h"
#include "Geometry.h"
#include "Integrator.h"
#include "Memory.h"
#include <unordered_map>


	extern double CorrectShadingNormal(const SurfaceIntersection& isect,
		const Vec3& wo, const Vec3& wi,
		TransportMode mode);


	struct EndpointIntersection : Intersection {
		union {
			const Camera* camera;
			const Light* light;
		};
		// EndpointIntersection Public Methods
		EndpointIntersection() : Intersection(), light(nullptr) {}
		EndpointIntersection(const Intersection& it, const Camera* camera)
			: Intersection(it), camera(camera) {}
		EndpointIntersection(const Camera* camera, const Ray& ray)
			: Intersection(ray.o, ray.time), camera(camera) {}
		EndpointIntersection(const Light* light, const Ray& r, const Vec3& nl)
			: Intersection(r.o, r.time), light(light) {
			mNormal = nl;
		}
		EndpointIntersection(const Intersection& it, const Light* light)
			: Intersection(it), light(light) {}
		EndpointIntersection(const Ray& ray)
			: Intersection(ray(1), ray.time), light(nullptr) {
			mNormal = Vec3(-1 * ray.d);
		}
	};

	enum class VertexType { Camera, Light, Surface};

	class BDPTIntegrator : public Integrator {
	public:
		// BDPTIntegrator Public Methods
		BDPTIntegrator(Sampler* sampler, int spp, int maxDepth, bool visualizeStrategies, bool visualizeWeights)
			: sampler(sampler), maxDepth(maxDepth), spp(spp), visualizeStrategies(visualizeStrategies), visualizeWeights(visualizeWeights) {}
		void Render(const Scene& scene, const Camera &camera) override;

	private:
		// BDPTIntegrator Private Data
		Sampler* sampler;
		const int maxDepth;
		const bool visualizeStrategies;
		const bool visualizeWeights;
		const int spp;
	};


	struct Vertex {
		// Vertex Public Data
		VertexType type;
		Vec3 beta;


		struct {
		// PBRT_HAVE_NONPOD_IN_UNIONS
			EndpointIntersection ei;
			SurfaceIntersection si;
		};
		bool delta = false;
		double pdfFwd = 0, pdfRev = 0;

		// Vertex Public Methods
		Vertex() : ei() {}
		Vertex(VertexType type, const EndpointIntersection& ei, const Vec3& beta)
			: type(type), beta(beta), ei(ei) {}
		Vertex(const SurfaceIntersection& si, const Vec3& beta)
			: type(VertexType::Surface), beta(beta), si(si) {}

		// Need to define these two to make compilers happy with the non-POD
		// objects in the anonymous union above.
		Vertex(const Vertex& v) { memcpy(this, &v, sizeof(Vertex)); }
		Vertex& operator=(const Vertex& v) {
			memcpy(this, &v, sizeof(Vertex));
			return *this;
		}

		static inline Vertex CreateCamera(const Camera* camera, const Ray& ray, const Vec3& beta);

		static inline Vertex CreateCamera(const Camera* camera, const Intersection& it, const Vec3& beta);

		static inline Vertex CreateLight(const Light* light, const Ray& ray, const Vec3& nLight, const Vec3& Le, double pdf);

		static inline Vertex CreateLight(const EndpointIntersection& ei, const Vec3& beta, double pdf);

		static inline Vertex CreateSurface(const SurfaceIntersection& si, const Vec3& beta, double pdf, const Vertex& prev);

		const Intersection& GetIntersection() const {
			switch (type) {
			case VertexType::Surface:
				return si;
			default:
				return ei;
			}
		}
		const Vec3& p() const { return GetIntersection().mPos; }
		const Vec3& ng() const { return GetIntersection().mNormal; }
		double time() const { return GetIntersection().mTime; }
		const Vec3& ns() const {
			if (type == VertexType::Surface)
				return si.mNormal;
			else
				return GetIntersection().mNormal;
		}
		bool IsOnSurface() const { return ng() != Vec3(); }
		Vec3 f(const Vertex& next, TransportMode mode) const {
			Vec3 wi = next.p() - p();
			if (wi.Dot(wi) == 0) return 0.;
			wi = wi.Norm();
			switch (type) {
			case VertexType::Surface:
				return si.mpBSDF->f(si.mOutDir, wi) *
					CorrectShadingNormal(si, si.mOutDir, wi, mode);
			}
		}
		bool IsConnectible() const {
			switch (type) {
			case VertexType::Light:
				return true;
			case VertexType::Camera:
				return true;
			case VertexType::Surface:
				return si.mpBSDF->bsdfType == BxDFType::BSDF_DIFFUSE;
			}
			return false;  // NOTREACHED
		}
		bool IsLight() const {
			return type == VertexType::Light ||
				(type == VertexType::Surface && si.pLight);
		}
		bool IsDeltaLight() const {
			return false;
		}
		bool IsInfiniteLight() const {
			return false;
		}
		Vec3 Le(const Scene& scene, const Vertex& v) const {
			if (!IsLight()) return Vec3(0.f);
			Vec3 w = v.p() - p();
			if (w.Dot(w) == 0) return 0.;
			Normalize(w);
			const std::shared_ptr<Light> light = si.pLight;
			return light->L(si, w);
		}

		double ConvertDensity(double pdf, const Vertex& next) const {
			// Return solid angle density if _next_ is an infinite area light
			if (next.IsInfiniteLight()) return pdf;
			Vec3 w = next.p() - p();
			if (w.Dot(w) == 0) return 0;
			double invDist2 = 1 / w.Dot(w);
			if (next.IsOnSurface())
				pdf *= std::fabs(Dot(next.ng(), w * std::sqrt(invDist2)));
			return pdf * invDist2;
		}
		double Pdf(const Scene& scene, const Vertex* prev,
			const Vertex& next) const {
			if (type == VertexType::Light) return PdfLight(scene, next);
			// Compute directions to preceding and next vertex
			Vec3 wn = next.p() - p();
			if (wn.Dot(wn) == 0) return 0;
			Normalize(wn);
			Vec3 wp;
			if (prev) {
				wp = prev->p() - p();
				if (wp.Dot(wp) == 0) return 0;
				Normalize(wp);
			}
			else if (type == VertexType::Camera) {
				
			}

			// Compute directional density depending on the vertex types
			double pdf = 0, unused;
			if (type == VertexType::Camera)
				ei.camera->PdfWe(ei.SpawnRay(wn), &unused, &pdf);
			else if (type == VertexType::Surface)
				pdf = si.mpBSDF->Pdf(wp, wn);
			// Return probability per unit area at vertex _next_
			return ConvertDensity(pdf, next);
		}
		double PdfLight(const Scene& scene, const Vertex& v) const {
			Vec3 w = v.p() - p();
			double invDist2 = 1 / w.Dot(w);
			w = w * std::sqrt(invDist2);
			double pdf;
			
			// Get pointer _light_ to the light source at the vertex
			const Light* light = type == VertexType::Light
				? ei.light
				: si.pLight.get();

			// Compute sampling density for non-infinite light sources
			double pdfPos, pdfDir;
			light->PdfLe(Ray(p(), w, Infinity, time()), ng(), &pdfPos, &pdfDir);
			pdf = pdfDir * invDist2;
			
			if (v.IsOnSurface()) pdf *= std::fabs(Dot(v.ng(), w));
			return pdf;
		}
		double PdfLightOrigin(const Scene& scene, const Vertex& v, const Distribution1D& lightDistr, const std::unordered_map<const Light*, size_t> & lightToDistrIndex) const {
			Vec3 w = v.p() - p();
			if (w.Dot(w) == 0) return 0.;
			Normalize(w);
			
			// Return solid angle density for non-infinite light sources
			double pdfPos, pdfDir, pdfChoice = 0;

			// Get pointer _light_ to the light source at the vertex
			const Light* light = type == VertexType::Light
				? ei.light
				: si.pLight.get();

			// Compute the discrete probability of sampling _light_, _pdfChoice_
			size_t index = lightToDistrIndex.find(light)->second;
			pdfChoice = lightDistr.DiscretePDF(index);

			light->PdfLe(Ray(p(), w, Infinity, time()), ng(), &pdfPos, &pdfDir);
			return pdfPos * pdfChoice;
		}
	};

	extern int GenerateCameraSubpath(
		const Scene& scene, 
		const Camera& camera,
		Sampler& sampler,
		MemoryPool &arena,
		int maxDepth,
		const Vec2& pFilm,
		Vertex* path);

	extern int GenerateLightSubpath(
		const Scene& scene, 
		Sampler& sampler, 
		MemoryPool &arena,
		int maxDepth,
		const std::unordered_map<const Light*, size_t>& lightToIndex,
		Vertex* path);

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
		double* misWeight = nullptr);


	// Vertex Inline Method Definitions
	inline Vertex Vertex::CreateCamera(const Camera* camera, const Ray& ray,
		const Vec3& beta) {
		return Vertex(VertexType::Camera, EndpointIntersection(camera, ray), beta);
	}

	inline Vertex Vertex::CreateCamera(const Camera* camera, const Intersection& it,
		const Vec3& beta) {
		return Vertex(VertexType::Camera, EndpointIntersection(it, camera), beta);
	}

	inline Vertex Vertex::CreateLight(const Light* light, const Ray& ray,
		const Vec3& Nl, const Vec3& Le,
		double pdf) {
		Vertex v(VertexType::Light, EndpointIntersection(light, ray, Nl), Le);
		v.pdfFwd = pdf;
		return v;
	}

	inline Vertex Vertex::CreateSurface(const SurfaceIntersection& si,
		const Vec3& beta, double pdf,
		const Vertex& prev) {
		Vertex v(si, beta);
		v.pdfFwd = prev.ConvertDensity(pdf, v);
		return v;
	}

	inline Vertex Vertex::CreateLight(const EndpointIntersection& ei,
		const Vec3& beta, double pdf) {
		Vertex v(VertexType::Light, ei, beta);
		v.pdfFwd = pdf;
		return v;
	}