#ifndef CAMERA_H
#define CAMERA_H
#include "Geometry.h"
#include "Film.h"
#include "Intersection.h"

struct CameraSample {
	Vec3 filmPoint;
};

class Camera {
public:
	Camera() {}
	void SetCamera(const Vec3 &_o, const Vec3 &_d, const Vec3 &_u, const Vec3 &_v, double _dis = 1.0f, double flov = 40.0f);
	void Init();
	int GenerateCameraRay(const CameraSample &sample, Ray &ray);
	Vec3 We(const Ray &ray) const {
		double PdfA = 1.0; // for the pinhole camera
		//double A = (film->LU - film->LL).length() * (film->RU - film->LU).length();
		//double A = film->width * film->heigh;
		double A = film->Area;
		double CosTheta = d.dot(ray.d);
		double CosTheta2 = CosTheta * CosTheta;
		double Value = dis * dis * PdfA / (A * CosTheta2 * CosTheta2);
		return Vec3(Value, Value, Value);
	}

	Vec3 Sample_Wi(const Intersection &isect, double *PdfW, Vec3 *wi) const {
		*wi = (o - isect.HitPoint);
		double distance = wi->length();
		wi->norm();
		double CosTheta = d.dot(-1 * (*wi));
		*PdfW = 1.0 * (distance * distance) / CosTheta;
		//*PdfW = 1.0 * (dis / CosTheta) * (dis / CosTheta) / CosTheta;
		return We(Ray(isect.HitPoint, -1 * (*wi)));
	}

	double PdfPos() const {
		return 1.0;
	}

	double PdfDir(const Ray &cameraRay) const {
		double AreaFilm = film->Area;
		double CosTheta = std::abs(d.dot(cameraRay.d));
		double Cos2Theta = CosTheta * CosTheta;
		return dis * dis / (AreaFilm * Cos2Theta * CosTheta);
	}

	Vec3 WordToScreen(const Vec3 &HitPoint, bool *inScreen) const {
		*inScreen = true;
		Vec3 dir = (HitPoint - o).norm();
		double CosCamera = d.dot(dir);
		double CameraToScreenDis = dis / CosCamera;
		Vec3 PointScreen = o + dir * CameraToScreenDis;

		Vec3 ScreenCenter = o + dis * d;
		Vec3 CenterToPoint = PointScreen - ScreenCenter;
		if (std::abs(CenterToPoint.dot(u)) > (film->LU - film->RU).length() / 2.f ||
			std::abs(CenterToPoint.dot(v)) > (film->LL - film->LU).length() / 2.f) {
			*inScreen = false;
			return Vec3(-1, -1, -1); //out of screen
		}

		Vec3 LU_P = PointScreen - film->LU;
		double dis_LUP = LU_P.length();
		double CosTheta = (film->LL - film->LU).norm().dot(LU_P.norm());
		double SinTheta = std::sqrt(1 - CosTheta * CosTheta);
		double pH = dis_LUP * CosTheta;
		double pW = dis_LUP * SinTheta;
		double alpha = pW / (film->RU - film->LU).length();
		double beta = pH / (film->LL - film->LU).length();
		
		int px = (int)(film->resX * alpha);
		int py = (int)(film->resY * beta);

		return Vec3(px, py);
	}

	Film* GetFilm() const  {
		return film;
	}

public:
	Film *film;
	Vec3 o, d;
private:
	Vec3 u, v;
	double dis, fovy;
	
};


#endif