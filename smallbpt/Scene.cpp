#include "Scene.h"
#include "Geometry.h"
#include "BSDF.h"
#include "Light.h"

//************************************Sphere scene************************************
//Sphere Scene::spheres[] = {//Scene: radius, position, emission, color, material
//	Sphere(1e5, Vec3(1e5 + 1,40.8,81.6), Vec3(),Vec3(.75,.25,.25),DIFF),//Left
//	Sphere(1e5, Vec3(-1e5 + 99,40.8,81.6),Vec3(),Vec3(.25,.25,.75),DIFF),//Right
//	Sphere(1e5, Vec3(50,40.8, 1e5),     Vec3(),Vec3(.75,.75,.75),DIFF),//Back
//	//Sphere(1e5, Vec3(50,40.8,-1e5 + 170), Vec3(),Vec3(), DIFF),//Front
//	Sphere(1e5, Vec3(50, 1e5, 81.6),    Vec3(),Vec3(.75,.75,.75),DIFF),//Bottom
//	Sphere(1e5, Vec3(50,-1e5 + 81.6,81.6),Vec3(),Vec3(.75,.75,.75),DIFF),//Top
//	Sphere(16.5,Vec3(27,16.5,47),       Vec3(),Vec3(1,1,1)*.999, SPEC),//Mirror
//	Sphere(16.5,Vec3(73,16.5,78),       Vec3(),Vec3(1,1,1)*.999, REFR),//Glass
//	//Sphere(16.5,Vec3(73,16.5,90),       Vec3(),Vec3(1,1,1)*.999, DIFF),
//	Sphere(8.0, Vec3(50,81.6 - 16.5,81.6),Vec3(0.30,0.30,0.30) * 100,  Vec3(), DIFF),//Lite
//};
//
//int Scene::numSpheres = sizeof(spheres) / sizeof(Sphere);






//*****************************************Sphere scene*****************************************

std::vector<Shape*> Scene::shapes = {
	new Sphere(1e5, Vec3(1e5 + 1,40.8,81.6), Vec3(),Vec3(.75,.25,.25),DIFF),//Left
	new Sphere(1e5, Vec3(-1e5 + 99,40.8,81.6),Vec3(),Vec3(.25,.25,.75),DIFF),//Right
	new Sphere(1e5, Vec3(50,40.8, 1e5),     Vec3(),Vec3(.75,.75,.75),DIFF),//Back
	new Sphere(1e5, Vec3(50, 1e5, 81.6),    Vec3(),Vec3(.75,.75,.75),DIFF),//Bottom
	new Sphere(1e5, Vec3(50,-1e5 + 81.6,81.6),Vec3(),Vec3(.75,.75,.75),DIFF),//Top
	new Sphere(16.5,Vec3(27,16.5,47),       Vec3(),Vec3(1,1,1) * .999, SPEC),//Mirror
	new Sphere(16.5,Vec3(73,16.5,78),       Vec3(),Vec3(1,1,1) * .999, REFR),//Glass
	new Sphere(8.0, Vec3(50,81.6 - 16.5,81.6),Vec3(0.30,0.30,0.30) * 100,  Vec3(), DIFF),//Lite
};

std::vector<Light*> Scene::lights = {
	new SphereLight(new Sphere(8.0, Vec3(50,81.6 - 16.5,81.6),Vec3(0.30,0.30,0.30) * 100,  Vec3(), DIFF))
};

//*****************************************Triangle scene*****************************************
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
Vec3 normalBottom = Vec3(0, 1, 0);
Vec3 normalTop = Vec3(0, -1, 0);
Vec3 lightNormal = Vec3(0, -1, 0);

Vec3 lightP0 = Vec3(0.25f, 0.965f, 0.25f);
Vec3 lightP1 = Vec3(0.25f, 0.965f, -0.25f);
Vec3 lightP2 = Vec3(-0.25f, 0.965f, -0.25f);
Vec3 lightP3 = Vec3(-0.25f, 0.965f, 0.25f);

//std::vector<Shape*> Scene::shapes = {
//	//left
//	new Triangle(p0, p1, p2, normalLeft, Vec3(.75f, .25f, .25f), Vec3(), DIFF),
//	new Triangle(p2, p3, p0, normalLeft, Vec3(.75f, .25f, .25f), Vec3(), DIFF),
//
//	//right
//	new Triangle(p4, p5, p6, normalRight, Vec3(.25f, .25f, .75f), Vec3(), DIFF),
//	new Triangle(p4, p6, p7, normalRight, Vec3(.25f, .25f, .75f), Vec3(), DIFF),
//
//	//back
//	new Triangle(p1, p5, p6, normalBack, Vec3(.75f, .75f, .75f), Vec3(), DIFF),
//	new Triangle(p1, p6, p2, normalBack, Vec3(.75f, .75f, .75f), Vec3(), DIFF),
//
//	//bottom
//	new Triangle(p0, p4, p5, normalBottom, Vec3(.75f, .75f, .75f), Vec3(), DIFF),
//	new Triangle(p0, p5, p1, normalBottom, Vec3(.75f, .75f, .75f), Vec3(), DIFF),
//
//	//top
//	new Triangle(p3, p7, p6, normalTop, Vec3(.75f, .75f, .75f), Vec3(), DIFF),
//	new Triangle(p3, p6, p2, normalTop, Vec3(.75f, .75f, .75f), Vec3(), DIFF),
//
//	//light
//	new Triangle(lightP0, lightP1, lightP2, lightNormal, Vec3(), Vec3(0.3f, 0.3f, 0.3f) * 85, DIFF),
//	new Triangle(lightP0, lightP2, lightP3, lightNormal, Vec3(), Vec3(0.3f, 0.3f, 0.3f) * 85, DIFF)
//};
//
//std::vector<Light*> Scene::lights = {
//	new AreaLight(new Triangle(lightP0, lightP1, lightP2, lightNormal, Vec3(), Vec3(0.3f, 0.3f, 0.3f) * 85, DIFF)),
//	new AreaLight(new Triangle(lightP0, lightP2, lightP3, lightNormal, Vec3(), Vec3(0.3f, 0.3f, 0.3f) * 85, DIFF))
//};

bool Scene::Intersect(const Ray &r, Intersection* isect) const {

	double t = Inf;
	Shape* pShape = nullptr;
	for (int i = 0; i < shapes.size(); i++) {
		Intersection isection;
		double tt;
		shapes[i]->Intersect(r, &isection, &tt);
		if (tt > 0  && t > tt) {
			t = tt;
			*isect = isection;
			pShape = shapes[i];
		}
	}

	if (t < Inf) {
		isect->mPos = r.o + t * r.d;
		isect->mSurfaceNormal = pShape->GetNormal(isect->mPos);
		isect->mNormal = isect->mSurfaceNormal.Dot(r.d) < 0 ? isect->mSurfaceNormal : -1 * isect->mSurfaceNormal;
		if (pShape->ReflectType() == DIFF) {
			isect->mpBSDF = std::make_shared<LambertianBSDF>(isect->mNormal, isect->mSurfaceNormal, pShape->Color());
		}
		else if (pShape->ReflectType() == SPEC) {
			isect->mpBSDF = std::make_shared<SpecularReflection>(isect->mNormal, isect->mSurfaceNormal, pShape->Color());
		}
		else if (pShape->ReflectType() == REFR) {
			isect->mpBSDF = std::make_shared<SpecularTransmission>(isect->mNormal, isect->mSurfaceNormal, pShape->Color(), 1.0, 1.5);
		}
		isect->mOutDir = -1 * r.d;
		isect->mIsDelta = isect->mpBSDF->IsDelta();
		isect->IsLight = (pShape->Emission() != Vec3());
		if (isect->IsLight) {
			isect->pLight = pShape;
		}
	}
	return t < Inf;
}

bool Scene::Intersect(const Ray& r) const {
	double t = Inf;
	for (int i = 0; i < shapes.size(); i++) {
		Intersection isection;
		double tt;
		shapes[i]->Intersect(r, &isection, &tt);
		if (tt > 0 && t > tt) {
			t = tt;
		}
	}
	return t > r.tmin && t < r.tmax;
}

Light* Scene::SampleOneLight(double* pdfLight, double u) const
{
	int lightCount = lights.size();
	int index = std::min(int(lightCount * u), lightCount - 1);
	*pdfLight = 1.0 / lightCount;
	return lights[index];
}
