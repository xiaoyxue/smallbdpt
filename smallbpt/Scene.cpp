#include "Scene.h"
#include "Geometry.h"
#include "BSDF.h"
#include "Light.h"

Sphere Scene::spheres[] = {//Scene: radius, position, emission, color, material
	Sphere(1e5, Vec3(1e5 + 1,40.8,81.6), Vec3(),Vec3(.75,.25,.25),DIFF),//Left
	Sphere(1e5, Vec3(-1e5 + 99,40.8,81.6),Vec3(),Vec3(.25,.25,.75),DIFF),//Right
	Sphere(1e5, Vec3(50,40.8, 1e5),     Vec3(),Vec3(.75,.75,.75),DIFF),//Back
	//Sphere(1e5, Vec3(50,40.8,-1e5 + 170), Vec3(),Vec3(), DIFF),//Front
	Sphere(1e5, Vec3(50, 1e5, 81.6),    Vec3(),Vec3(.75,.75,.75),DIFF),//Bottom
	Sphere(1e5, Vec3(50,-1e5 + 81.6,81.6),Vec3(),Vec3(.75,.75,.75),DIFF),//Top
	Sphere(16.5,Vec3(27,16.5,47),       Vec3(),Vec3(1,1,1)*.999, SPEC),//Mirror
	Sphere(16.5,Vec3(73,16.5,78),       Vec3(),Vec3(1,1,1)*.999, REFR),//Glass
	//Sphere(16.5,Vec3(73,16.5,90),       Vec3(),Vec3(1,1,1)*.999, DIFF),
	Sphere(8.0, Vec3(50,81.6 - 16.5,81.6),Vec3(0.30,0.30,0.30) * 100,  Vec3(), DIFF),//Lite
};

int Scene::numSpheres = sizeof(spheres) / sizeof(Sphere);

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

bool Scene::Intersect(const Ray &r, double &t, Intersection& isect) const {
	double d;
	t = Inf;
	int id;
	Shape* pShape = nullptr;
	for (int i = 0; i < shapes.size(); ++i) {
		if ((d = shapes[i]->IntersectP(r)) && d < t) { t = d; id = i; pShape = shapes[id]; }
	}

	if (t < Inf) {
		isect.HitPoint = r.o + t * r.d;
		isect.SurfaceNormal = shapes[id]->GetNormal(isect.HitPoint);
		isect.Normal = isect.SurfaceNormal.dot(r.d) < 0 ? isect.SurfaceNormal : -1 * isect.SurfaceNormal;
		if (pShape->ReflectType() == DIFF) {
			isect.bsdf = std::make_shared<LambertianBSDF>(isect.Normal, isect.SurfaceNormal, pShape->Color());
		}
		else if (pShape->ReflectType() == SPEC) {
			isect.bsdf = std::make_shared<SpecularReflection>(isect.Normal, isect.SurfaceNormal, pShape->Color());
		}
		else if (pShape->ReflectType() == REFR) {
			isect.bsdf = std::make_shared<SpecularTransmission>(isect.Normal, isect.SurfaceNormal, pShape->Color(), 1.0, 1.5);
		}
		isect.wo = -1 * r.d;
		isect.Delta = isect.bsdf->IsDelta();
		isect.IsLight = (shapes[id]->Emission() != Vec3());
	}
	return t < Inf;
}

bool Scene::Intersect(const Ray& r) const {
	double d, inf = 1e20, tmax = r.tmax, tmin = r.tmin, t = r.tmax;
	for (int i = 0; i < shapes.size(); ++i) {
		if ((d = shapes[i]->IntersectP(r)) && d < t && d > tmin)
		{
			t = d;
		}
	}
	return t < r.tmax&& t > r.tmin;
}

Light* Scene::SampleOneLight(double* pdfLight, double u) const
{
	int lightCount = lights.size();
	int index = std::min(int(lightCount * u), lightCount - 1);
	*pdfLight = 1.0 / lightCount;
	return lights[index];
}
