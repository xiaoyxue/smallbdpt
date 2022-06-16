#ifndef BDPT_H
#define BDPT_H
#include "PathVertex.h"
#include "Integrator.h"
#include "Camera.h"
#include "Sampler.h"
#include "Intersect.h"
#include <vector>
#include "Geometry.h"
#include "PathVertex.h"

template <typename Type>
class ScopedAssignment {
public:
	// ScopedAssignment Public Methods
	ScopedAssignment(Type *target = nullptr, Type value = Type())
		: target(target) {
		if (target) {
			backup = *target;
			*target = value;
		}
	}
	~ScopedAssignment() {
		if (target) *target = backup;
	}
	ScopedAssignment(const ScopedAssignment &) = delete;
	ScopedAssignment &operator=(const ScopedAssignment &) = delete;
	ScopedAssignment &operator=(ScopedAssignment &&other) {
		target = other.target;
		backup = other.backup;
		other.target = nullptr;
		return *this;
	}

private:
	Type *target, backup;
};

class BidirectionalPathTracing : public Integrator {
public:
	BidirectionalPathTracing() {}
	BidirectionalPathTracing(Sampler *smp, Camera *cam, Film *_film, int maxdepth, int samplePerPixel,
		bool _visualizeStrategies, bool _visualizeWeight) :
		sampler(smp), camera(cam), film(_film), maxDepth(maxdepth), spp(samplePerPixel),
		visualizeStrategies(_visualizeStrategies), visualizeWeight(_visualizeWeight){}
	void Render();
private:
	bool visualizeStrategies;
	bool visualizeWeight;

	int spp, maxDepth;
	Film *film;
	Camera *camera;
	Sampler *sampler;
	//std::vector<PathVertex> LightPath;
};

int GenerateLightPath(Sampler &sampler, std::vector<PathVertex> &LightPath, int maxdepth);
int GenerateCameraPath(Sampler &sampler, std::vector<PathVertex> &CameraPath, const Camera &camera, 
	const Ray &cameraRay, int maxdepth);
int Trace(const Ray &ray, Vec3 Throughput, double PdfFwd, Sampler &sampler, 
	std::vector<PathVertex> &Path, int depth, int maxDepth);
Vec3 ConnectBDPT(std::vector<PathVertex> &LightPath, std::vector<PathVertex> &CameraPath,
	int s, int t, Camera &camera, Sampler &sampler, Vec3 *pRaster, bool *inScreen, double *MISRecord);
double ConvertSolidToArea(double PdfW, const PathVertex &Vertex, const PathVertex &nxt);
double G(const PathVertex &VertexA, const PathVertex &VertexB);
bool IsConnectible(const Vec3 &PointA, const Vec3 &PointB);
double MISWeight(std::vector<PathVertex> &LightPath, std::vector<PathVertex> &CameraPath,
	int s, int t, PathVertex &sampled);
double MISWeight2(const std::vector<PathVertex> &LightPath, const std::vector<PathVertex> &CameraPath,
	int s, int t);
double MISWeight3(std::vector<PathVertex> &LightPath, std::vector<PathVertex> &CameraPath,
	int s, int t, PathVertex &sampled);
double PathPdf(const std::vector<PathVertex> &LightPath, const std::vector<PathVertex> &CameraPath,
	int s, int t, bool specialPath = false);
double MISWeight4(const std::vector<PathVertex> &LightPath, const std::vector<PathVertex> &CameraPath,
	int s, int t);

double Path_Pdf(const std::vector<PathVertex> &Path, int s, int t);
#endif
