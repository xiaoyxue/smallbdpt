//#include "PathTracing.h"
//#include "Scene.h"
//#include "Intersection.h"
//#include "Smallbpt.h"
//#include "stdio.h"
//#include <stdlib.h>
//#include <iostream>
//
//void PathTracing::Render(const Scene &scene, const Camera &camera) {
//	int64_t resX = camera.GetFilm()->resX;
//	int64_t resY = camera.GetFilm()->resY;
//	for (int y = 0; y < resY; ++y) {
//		fprintf(stderr, "\rRendering %5.2f%%",
//			(1.0 * y * resX) / resX / resY * 100.0);
//#pragma omp parallel for schedule(dynamic, 1)
//		for (int x = 0; x < resX; ++x) {
//			Vec3 r;
//			for (int s = 0; s < spp; ++s) {
//				double u = sampler->Get1D() - 0.5f;
//				double v = sampler->Get1D() - 0.5f;
//				double ImageX = x + 0.5f + u;
//				double ImageY = y + 0.5f + v;
//				double alpha = ImageX / (double)(resX);
//				double beta = ImageY / (double)(resY);
//				Vec3 p0 = camera.GetFilm()->LU + alpha * (camera.GetFilm()->RU - camera.GetFilm()->LU);
//				Vec3 p1 = camera.GetFilm()->LL + alpha * (camera.GetFilm()->RL - camera.GetFilm()->LL);
//				Vec3 p = p0 + beta * (p1 - p0);
//				Vec3 d = (p - camera.o).norm();
//
//				//r = r + radiance(Ray(camera->o + d * 140, d), 0)*(1. / spp);
//			
//				r = r + Li(scene, camera, Ray(camera.o + d * 140, d), *sampler)*(1. / spp);
//
//			}
//			r = Vec3(clamp(r.x), clamp(r.y), clamp(r.z));
//			camera.GetFilm()->AddSample(Vec3(x, y), r);
//		}
//	}
//	camera.GetFilm()->WriteToImage();
//	//camera->film->WriteToVisualImage("visual_image.png");
//}
//
////smallpt version
//Vec3 PathTracing::radiance(const Scene &scene, const Camera& camera, const Ray &r, int depth, int E) {
//	double t;                               // distance to intersection
//	int id = 0;                               // id of intersected object
//	Intersection isect;
//	if (!scene.intersect(r, t, id, isect)) return Vec3(); // if miss, return black
//
//	//visual
//	if (visual) {
//		bool flag;
//		Vec3 pFilm = camera.WordToScreen(isect.HitPoint, &flag);
//		if (flag) {
//			camera.GetFilm()->AddVisualPlane2(pFilm, Vec3(depth * 0.1, 1, 0));
//		}
//	}
//
//	const Sphere &obj = Scene::spheres[id];        // the hit object
//	Vec3 x = r.o + r.d*t, n = (x - obj.p).norm(), nl = n.dot(r.d)<0 ? n : n*-1, f = obj.c;
//	double p = f.x>f.y && f.x>f.z ? f.x : f.y>f.z ? f.y : f.z; // max refl
//	if (++depth>3 || !p) if (sampler->Get1D()<p) f = f*(1 / p); else return obj.e*E;
//	if (obj.refl == DIFF) {                  // Ideal DIFFUSE reflection
//		double r1 = 2 * PI*sampler->Get1D(), r2 = sampler->Get1D(), r2s = sqrt(r2);
//		Vec3 w = nl, u = ((fabs(w.x)>.1 ? Vec3(0, 1) : Vec3(1)) % w).norm(), v = w%u;
//		Vec3 d = (u*cos(r1)*r2s + v*sin(r1)*r2s + w*sqrt(1 - r2)).norm();
//
//		// Loop over any lights
//		Vec3 e;
//		for (int i = 0; i< Scene::numSpheres; i++) {
//			const Sphere &s = Scene::spheres[i];
//			if (s.e.x <= 0 && s.e.y <= 0 && s.e.z <= 0) continue; // skip non-lights
//
//			Vec3 sw = s.p - x, su = ((fabs(sw.x)>.1 ? Vec3(0, 1) : Vec3(1)) % sw).norm(), sv = sw%su;
//			double cos_a_max = sqrt(1 - s.rad*s.rad / (x - s.p).dot(x - s.p));
//			double eps1 = sampler->Get1D(), eps2 = sampler->Get1D();
//			double cos_a = 1 - eps1 + eps1*cos_a_max;
//			double sin_a = sqrt(1 - cos_a*cos_a);
//			double phi = 2 * PI*eps2;
//			Vec3 l = su*cos(phi)*sin_a + sv*sin(phi)*sin_a + sw*cos_a;
//			l.norm();
//			if (scene.intersect(Ray(x, l), t, id, isect) && id == i) {  // shadow ray
//				double omega = 2 * PI*(1 - cos_a_max);
//				e = e + f.mult(s.e*l.dot(nl)*omega)*INV_PI;  // 1/pi for brdf
//			}
//		}
//
//		return obj.e*E + e + f.mult(radiance(scene, camera, Ray(x, d), depth, 0));
//	}
//	else if (obj.refl == SPEC)              // Ideal SPECULAR reflection
//		return obj.e + f.mult(radiance(scene, camera, Ray(x, r.d - n * 2 * n.dot(r.d)), depth));
//	/*
//	Ray reflRay(x, r.d - n * 2 * n.dot(r.d));     // Ideal dielectric REFRACTION
//	bool into = n.dot(nl)>0;                // Ray from outside going in?
//	double nc = 1, nt = 1.5, nnt = into ? nc / nt : nt / nc, ddn = r.d.dot(nl), cos2t;
//	if ((cos2t = 1 - nnt*nnt*(1 - ddn*ddn))<0)    // Total internal reflection
//		return obj.e + f.mult(radiance(reflRay, depth));
//	Vec3 tdir = (r.d*nnt - n*((into ? 1 : -1)*(ddn*nnt + sqrt(cos2t)))).norm();
//	double a = nt - nc, b = nt + nc, R0 = a*a / (b*b), c = 1 - (into ? -ddn : tdir.dot(n));
//	double Re = R0 + (1 - R0)*c*c*c*c*c, Tr = 1 - Re, P = .25 + .5*Re, RP = Re / P, TP = Tr / (1 - P);
//	return obj.e + f.mult(depth>0 ? (sampler->Get1D()<P ?   // Russian roulette
//		radiance(reflRay, depth)*RP : radiance(Ray(x, tdir), depth)*TP) :
//		radiance(reflRay, depth)*Re + radiance(Ray(x, tdir), depth)*Tr);*/
//	else if(obj.refl == REFR){
//		Vec3 tDir;
//		Vec3 refDir = r.d - n * 2 * n.dot(r.d);
//		double pdf;
//		Vec3 fr = isect.bsdf->Sample_f(isect.wo, &tDir, &pdf, sampler->Get2D());
//		Ray refRay(isect.HitPoint, refDir);
//		
//		bool into = n.dot(nl) > 0;                // Ray from outside going in?
//		double nc = 1, nt = 1.5, nnt = into ? nc / nt : nt / nc, ddn = r.d.dot(nl), cos2t;
//		if ((cos2t = 1 - nnt*nnt*(1 - ddn*ddn)) < 0)    // Total internal reflection
//			return obj.e + f.mult(radiance(scene, camera, refRay, depth));
//		Vec3 tdir = (r.d*nnt - n*((into ? 1 : -1)*(ddn*nnt + sqrt(cos2t)))).norm();
//		Ray tRay(isect.HitPoint + eps * tDir, tDir);
//		double a = nt - nc, b = nt + nc, R0 = a*a / (b*b), c = 1 - std::abs(-ddn);//c = 1 - (into ? -ddn : tdir.dot(n));
//		double Re = R0 + (1 - R0)*c*c*c*c*c, Tr = 1 - Re, P = .25 + .5*Re, RP = Re / P, TP = Tr / (1 - P);
//		//std::cout << " 2: " << Re << std::endl;
//		//if(tDir != tdir) std::cout << tDir << " " << tdir << " " << refDir<<std::endl;
//		//std::cout <<" " <<c << std::endl;
//		return obj.e + fr.mult(radiance(scene, camera, tRay, depth)) * std::abs(tDir.dot(n)) / pdf;
//		//return obj.e + f.mult(depth>0 ? (sampler->Get1D()<P ?   // Russian roulette
//			//radiance(refRay, depth)*RP : radiance(Ray(x, tdir), depth)*TP) :
//			//radiance(refRay, depth)*Re + radiance(Ray(x, tdir), depth)*Tr);
//	}
//}
//
//
//Vec3 PathTracing::Li(const Scene &scene, const Camera& camera, const Ray &ray, Sampler &sampler) {
//	Vec3 Throughput(1.0, 1.0, 1.0);
//	Vec3 L(0.0, 0.0, 0.0);
//	int bound = 0;
//	Ray r = ray;
//	bool isDelta = false;
//	while (true) {
//		//intersect the scene
//		double t; int id;
//		Intersection isect;
//		if (!scene.intersect(r, t, id, isect)) {
//			break;
//		}
//		//visual, debug
//		if (visual) {
//			bool flag;
//			Vec3 pFilm = camera.WordToScreen(isect.HitPoint, &flag);
//			if (flag) {
//				camera.GetFilm()->AddVisualPlane(pFilm, false);
//			}
//		}
//
//
//		if (bound == 0 || isDelta) {
//			L = L + Throughput * Scene::spheres[id].e;
//		}
//		
//		//direct lighting
//		//if (!isect.bsdf->IsDelta() && bound != 0) {
//		if (!isect.bsdf->IsDelta()){
//			L += DirectIllumination(scene, isect, Throughput, sampler);
//		}
//		++bound;
//		if (bound >= maxDepth) break;
//
//		//sample BSDF
//		double PdfW;
//		Vec3 wi;
//		Vec3 f = isect.bsdf->Sample_f(-1 * r.d, &wi, &PdfW, sampler.Get2D());
//		Throughput = Throughput * f * wi.dot(isect.Normal) / PdfW;
//		isDelta = isect.bsdf->IsDelta();
//
//		r.o = isect.HitPoint + wi * eps;
//		r.d = wi;
//
//		
//		Vec3 rrBeta = Throughput;
//		if (bound > 8) {
//			double q = std::max(0.5, 1 - rrBeta.maxComponentValue());
//			if (sampler.Get1D() < q) break;
//			Throughput = Throughput / (1 - q);
//		}
//	}
//
//	return L;
//}
//
//
//Vec3 DirectIllumination(const Scene &scene, const Intersection &isect, const Vec3 &Throughput, Sampler &sampler) {
//	Sphere &light = Scene::spheres[Scene::numSpheres - 1];
//	Vec3 localZ = (light.p - isect.HitPoint).norm(), localX, localY;
//	CoordinateSystem(localZ, &localX, &localY);
//	
//	double SinThetaMax = light.rad / (light.p - isect.HitPoint).length();
//	double CosThetaMax = std::sqrt(1 - SinThetaMax * SinThetaMax);
//	Vec3 wi = UniformSampleCone(sampler.Get2D(), CosThetaMax, localX, localY, localZ);
//	double PdfW = UniformConePdf(CosThetaMax);
//
//	Ray shadowRay(isect.HitPoint, wi);
//	double t; int id;
//	Intersection isec;
//	Vec3 f = isect.bsdf->f(isect.wo, wi);
//	if (!scene.intersect(shadowRay, t, id, isec) || id != Scene::numSpheres - 1) return Vec3(0.0, 0.0, 0.0);
//
//	return Throughput * f * wi.dot(isect.Normal) * light.e / PdfW;
//}