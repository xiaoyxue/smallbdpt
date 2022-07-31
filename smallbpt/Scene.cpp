#include "Scene.h"
#include "Geometry.h"
#include "BSDF.h"
#include "Light.h"

//Points
Vec3 p0(-1, -1, 1);
Vec3 p1(-1, -1, -1);
Vec3 p2(-1, 1, -1);
Vec3 p3(-1, 1, 1);
Vec3 p4(1, -1, 1);
Vec3 p5(1, -1, -1);
Vec3 p6(1, 1, -1);
Vec3 p7(1, 1, 1);

Vec3 normalLeft = Vec3(1, 0, 0);
Vec3 normalRight = Vec3(-1, 0, 0);
Vec3 normalBack = Vec3(0, 0, 1);
Vec3 normalBotom = Vec3(0, 1, 0);
Vec3 normalTop = Vec3(0, -1, 0);
Vec3 lightNormal = Vec3(0, -1, 0);

//Light
Vec3 lightP0 = Vec3(0.25f, 0.965f, 0.25f);
Vec3 lightP1 = Vec3(0.25f, 0.965f, -0.25f);
Vec3 lightP2 = Vec3(-0.25f, 0.965f, -0.25f);
Vec3 lightP3 = Vec3(-0.25f, 0.965f, 0.25f);

std::vector<std::shared_ptr<Triangle>> Scene::triangles = {
	//Left
	std::shared_ptr<Triangle>(new Triangle(p0, p1, p2, normalLeft, Vec3(.75f, .25f, .25f), Vec3(), DIFF)),
	std::shared_ptr<Triangle>(new Triangle(p2, p3, p0, normalLeft, Vec3(.75f, .25f, .25f), Vec3(), DIFF)),

	//Right
	std::shared_ptr<Triangle>(new Triangle(p4, p5, p6, normalRight, Vec3(.25f, .25f, .75f), Vec3(), DIFF)),
	std::shared_ptr<Triangle>(new Triangle(p4, p6, p7, normalRight, Vec3(.25f, .25f, .75f), Vec3(), DIFF)),

	//Back
	std::shared_ptr<Triangle>(new Triangle(p1, p5, p6, normalBack, Vec3(.75f, .75f, .75f), Vec3(), DIFF)),
	std::shared_ptr<Triangle>(new Triangle(p1, p6, p2, normalBack, Vec3(.75f, .75f, .75f), Vec3(), DIFF)),

	//Bottom
	std::shared_ptr<Triangle>(new Triangle(p0, p4, p5, normalBotom, Vec3(.75f, .75f, .75f), Vec3(), DIFF)),
	std::shared_ptr<Triangle>(new Triangle(p0, p5, p1, normalBotom, Vec3(.75f, .75f, .75f), Vec3(), DIFF)),

	//Top
	std::shared_ptr<Triangle>(new Triangle(p3, p7, p6, normalTop, Vec3(.75f, .75f, .75f), Vec3(), DIFF)),
	std::shared_ptr<Triangle>(new Triangle(p3, p6, p2, normalTop, Vec3(.75f, .75f, .75f), Vec3(), DIFF)),

	//Light
	std::shared_ptr<Triangle>(new Triangle(lightP0, lightP1, lightP2, lightNormal, Vec3(), Vec3(0.3f, 0.3f, 0.3f) * 85, DIFF)),
	std::shared_ptr<Triangle>(new Triangle(lightP0, lightP2, lightP3, lightNormal, Vec3(), Vec3(0.3f, 0.3f, 0.3f) * 85, DIFF))
};

std::vector<std::shared_ptr<Light>> Scene::lights = {
	std::shared_ptr<Light>(new AreaLight(std::shared_ptr<Triangle>(new Triangle(lightP0, lightP1, lightP2, lightNormal, Vec3(), Vec3(0.3f, 0.3f, 0.3f) * 85, DIFF)), Vec3(0.3f, 0.3f, 0.3f) * 85)),
	std::shared_ptr<Light>(new AreaLight(std::shared_ptr<Triangle>(new Triangle(lightP0, lightP2, lightP3, lightNormal, Vec3(), Vec3(0.3f, 0.3f, 0.3f) * 85, DIFF)), Vec3(0.3f, 0.3f, 0.3f) * 85))
};


bool Scene::intersect(const Ray &r, double &t, Intersection& isect) const {

	double tmin = Inf;
	/*Intersection isect;*/
	std::shared_ptr<Triangle> p = nullptr;
	for (auto triangle : triangles) {
		Intersection isection;
		double t;
		triangle->intersect(r, &isection, &t);
		if (tmin > t) {
			tmin = t;
			isect = isection;
			p = triangle;
		}
	}

	if (tmin < Inf) {
		isect.HitPoint = r.o + t * r.d;
		isect.Normal = isect.SurfaceNormal.dot(r.d) < 0 ? isect.SurfaceNormal : -1 * isect.SurfaceNormal;
		if (p->refl == DIFF) {
			isect.bsdf = std::make_shared<LambertianBSDF>(isect.Normal, isect.SurfaceNormal, p->color);
		}
		else if (p->refl == SPEC) {
			isect.bsdf = std::make_shared<SpecularReflection>(isect.Normal, isect.SurfaceNormal, p->color);
		}
		else if (p->refl == REFR) {
			isect.bsdf = std::make_shared<SpecularTransmission>(isect.Normal, isect.SurfaceNormal, p->color, 1.0, 1.5);
		}
		isect.wo = -1 * r.d;
		isect.Delta = isect.bsdf->IsDelta();
		isect.IsLight = (p->emissive != Vec3());
	}
	return tmin < Inf;
}

bool Scene::intersect(const Ray &r) const {
	for (auto tringle : triangles) {
		if (tringle->intersect(r)) {
			return true;
		}
	}
	return false;
}

std::shared_ptr<Light> Scene::SampleOneLight(double* pdfLight, double u) const
{
	int lightCount = lights.size();
	int index = std::min((int)(lightCount * u), lightCount - 1);
	*pdfLight = 1.0 / lightCount;
	return lights[index];
}
