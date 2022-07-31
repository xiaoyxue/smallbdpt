#ifndef BDPT_H
#define BDPT_H
#include "PathVertex.h"
#include "Integrator.h"
#include "Camera.h"
#include "Sampler.h"
#include "Intersection.h"
#include <vector>
#include "Geometry.h"
#include "PathVertex.h"
#include "ScopedAssignment.h"

class BidirectionalPathTracing : public Integrator {
public:
	BidirectionalPathTracing() {}
	BidirectionalPathTracing(Sampler *pSampler,int maxDepth, int spp, bool visualizeStrategies, bool visualizeWeight) :
		mpSampler(pSampler), mMaxDepth(maxDepth), mSpp(spp),
		mVisualizeStrategies(visualizeStrategies), mVisualizeWeight(visualizeWeight){}
	void Render(const Scene &scene, const Camera& camera);
private:
	bool mVisualizeStrategies;
	bool mVisualizeWeight;

	int mSpp, mMaxDepth;
	Sampler *mpSampler;
};

int GenerateLightPath(const Scene& scene, Sampler &sampler, std::vector<PathVertex> &LightPath, int maxdepth);

int GenerateCameraPath(const Scene& scene, const Camera& camera, Sampler &sampler, std::vector<PathVertex> &CameraPath, const Ray &cameraRay, int maxdepth);

int Trace(const Scene& scene, const Ray &ray, Vec3 Throughput, double PdfFwd, Sampler &sampler,
	std::vector<PathVertex> &Path, int depth, int maxDepth);

Vec3 ConnectBDPT(const Scene& scene, const Camera& camera, Sampler& sampler, 
	std::vector<PathVertex>& LightPath, std::vector<PathVertex>& CameraPath, int s, int t, Vec3* pRaster, bool* inScreen, double* MISRecord);

double ConvertSolidToArea(double PdfW, const PathVertex &Vertex, const PathVertex &nxt);

double G(const PathVertex &VertexA, const PathVertex &VertexB);

bool IsConnectable(const Scene& scene, const Vec3 &PointA, const Vec3 &PointB);

double MISWeight(std::vector<PathVertex> &LightPath, std::vector<PathVertex> &CameraPath,
	int s, int t, PathVertex &sampled);

double MISWeight2(std::vector<PathVertex> &LightPath, std::vector<PathVertex> &CameraPath,
	int s, int t, PathVertex &sampled);

double Path_Pdf(const std::vector<PathVertex> &Path, int s, int t);
#endif
