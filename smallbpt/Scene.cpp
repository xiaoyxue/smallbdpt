#include "Scene.h"
#include "Geometry.h"
#include "BSDF.h"

Sphere Scene::spheres[] = {//Scene: radius, position, emission, color, material
	Sphere(1e5, Vec3(1e5 + 1,40.8,81.6), Vec3(),Vec3(.75,.25,.25),DIFF),//Left
	Sphere(1e5, Vec3(-1e5 + 99,40.8,81.6),Vec3(),Vec3(.25,.25,.75),DIFF),//Rght
	Sphere(1e5, Vec3(50,40.8, 1e5),     Vec3(),Vec3(.75,.75,.75),DIFF),//Back
	//Sphere(1e5, Vec3(50,40.8,-1e5 + 170), Vec3(),Vec3(), DIFF),//Frnt
	Sphere(1e5, Vec3(50, 1e5, 81.6),    Vec3(),Vec3(.75,.75,.75),DIFF),//Bottm
	Sphere(1e5, Vec3(50,-1e5 + 81.6,81.6),Vec3(),Vec3(.75,.75,.75),DIFF),//Top
	Sphere(16.5,Vec3(27,16.5,47),       Vec3(),Vec3(1,1,1)*.999, SPEC),//Mirr
	Sphere(16.5,Vec3(73,16.5,78),       Vec3(),Vec3(1,1,1)*.999, REFR),//Glas
	//Sphere(16.5,Vec3(73,16.5,90),       Vec3(),Vec3(1,1,1)*.999, DIFF),
	Sphere(8.0, Vec3(50,81.6 - 16.5,81.6),Vec3(0.30,0.30,0.30) * 100,  Vec3(), DIFF),//Lite
};

int Scene::numSpheres = sizeof(spheres) / sizeof(Sphere);


bool Scene::intersect(const Ray &r, double &t, int &id, Intersection& isect) const {
	double n = sizeof(spheres) / sizeof(Sphere), d, inf = t = 1e20;
	for (int i = int(n); i--;) if ((d = spheres[i].intersect(r)) && d < t) { t = d; id = i; }
#ifdef _DEBUG
	isect.id = id;
#endif
	if (t < inf) {
		isect.HitPoint = r.o + t * r.d;
		isect.SurfaceNormal = (isect.HitPoint - spheres[id].p).norm();
		isect.Normal = isect.SurfaceNormal.dot(r.d) < 0 ? isect.SurfaceNormal : -1 * isect.SurfaceNormal;
		if (spheres[id].refl == DIFF) {
			isect.bsdf = std::make_shared<LambertianBSDF>(isect.Normal, isect.SurfaceNormal, spheres[id].c);
		}
		else if (spheres[id].refl == SPEC) {
			isect.bsdf = std::make_shared<SpecularReflection>(isect.Normal, isect.SurfaceNormal, spheres[id].c);
		}
		else if (spheres[id].refl == REFR) {
			isect.bsdf = std::make_shared<SpecularTransmission>(isect.Normal, isect.SurfaceNormal, spheres[id].c, 1.0, 1.5);
		}
		isect.wo = -1 * r.d;
		isect.Delta = isect.bsdf->IsDelta();
		isect.IsLight = id == numSpheres - 1;
	}
	return t < inf;
}

bool Scene::intersect(const Ray &r) const {
	double d, inf = 1e20, tmax = r.tmax, tmin = r.tmin, t = r.tmax;
	int id = -1;
	double n = sizeof(spheres) / sizeof(Sphere);
	for (int i = int(n); i--;) if ((d = spheres[i].intersect(r)) && d < t && d > tmin) { t = d; id = i; }
	return t < r.tmax && t > r.tmin;
}