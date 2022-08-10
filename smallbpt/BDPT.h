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
#include "BSDF.h"


class BidirectionalPathTracing : public Integrator {
public:
	BidirectionalPathTracing() {}
	BidirectionalPathTracing(Sampler *pSampler, int spp, int maxDepth, bool visualizeStrategies, bool visualizeWeight) :
		mpSampler(pSampler), mMaxDepth(maxDepth), mSpp(spp),
		mVisualizeStrategies(visualizeStrategies), mVisualizeWeight(visualizeWeight){}
	void Render(const Scene &scene, const Camera& camera);
private:
	bool mVisualizeStrategies;
	bool mVisualizeWeight;

	int mSpp, mMaxDepth;
	Sampler *mpSampler;
};

int GenerateLightPath(
	const Scene					&scene, 
	Sampler						&sampler, 
	std::vector<PathVertex>		&lightPath, 
	int							maxdepth);

int GenerateCameraPath(
	const Scene					&scene, 
	const Camera				&camera,
	Sampler						&sampler, 
	std::vector<PathVertex>		&cameraPath, 
	const Ray					&cameraRay,
	int							maxdepth);

int Trace(
	const Scene					&scene, 
	const Ray					&ray, 
	Vec3						throughput, 
	double						pdfFwd, 
	Sampler						&sampler,
	std::vector<PathVertex>		&path, 
	int							depth, 
	int							maxDepth,
	TransportMode				mode);

Vec3 ConnectBDPT(
	const Scene					&scene, 
	const Camera				&camera, 
	Sampler						&sampler, 
	std::vector<PathVertex>		&lightPath, 
	std::vector<PathVertex>		&cameraPath, 
	int							s, 
	int							t, 
	Vec3						*pRaster, 
	bool						*inScreen, 
	double						*misRecord);

double ConvertSolidToArea(
	double						pdfW, 
	const PathVertex			&vertex, 
	const PathVertex			&nextVertex);

double G(
	const PathVertex			&vertexA, 
	const PathVertex			&vertexB);

bool IsConnectable(
	const Scene					&scene, 
	const Vec3					&pointA, 
	const Vec3					&pointB);

double MISWeight(
	std::vector<PathVertex>		&lightPath, 
	std::vector<PathVertex>		&cameraPath,
	int							s, 
	int							t, 
	PathVertex					&sampled);

double MISWeight2(
	std::vector<PathVertex>		&lightPath, 
	std::vector<PathVertex>		&cameraPath,
	int							s, 
	int							t,
	PathVertex					&sampled);

double Path_Pdf(
	const std::vector<PathVertex>	&path, 
	int								s, 
	int								t);

double MISWeight3(
	const Scene& scene,
	const Camera& camera,
	Sampler& sampler,
	std::vector<PathVertex>& lightPath,
	std::vector<PathVertex>& cameraPath,
	int							s,
	int							t,
	const double				GConnect,
	std::vector<BDPTMISNode>& misNode);

#endif
