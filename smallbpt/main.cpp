#include <iostream>
#include "Scene.h"
#include "Camera.h"
#include "Geometry.h"
#include "Film.h"
#include "Sampler.h"
#include "Integrator.h"
#include "LightTracing.h"
#include "BDPT.h"
#include "PT.h"


void SceneOne() {
	int width = 1024 , height = 768;

	std::string filename = "Result/Sphere3.png";
	Film film(width, height, filename);
	Camera camera;
	Vec3 camPos(50, 52, 295.6), d(0, -0.042612, -1);
	d.Norm();
	Vec3 u = Vec3(width * .5135 / height).Norm();
	Vec3 v = (u.Cross(d)).Norm();
	std::cout << camPos + d << std::endl << v << std::endl;
	camera.SetCamera(camPos, d, u, v, 1.0, 28.7993);
	camera.film = &film;
	camera.Init();
	Sampler sampler;
	Scene scene;

	//LightTracing lt(&sampler, &camera, 8, 32);
	//lt.Render(scene, camera);

	BidirectionalPathTracing bpt(&sampler, 15, 32, false, false);
	bpt.Render(scene, camera);

	//PathTracing pt(&sampler, 128, 15);
	//pt.Render(scene, camera);
}


void SceneTwo() {
	int width = 1024, height = 1024;

	std::string filename = "Result/BDPT_test9.png";
	Film film(width, height, filename);
	Camera camera;
	Vec3 camPos(0, 0, 3), d(0, 0, -1);
	d.Norm();
	Vec3 u = Vec3(1, 0, 0);
	Vec3 v = Vec3(0, 1, 0);
	double fovy = 53.13010235415597;
	double filmDis = 1;
	camera.SetCamera(camPos, d, u, v, filmDis, fovy);
	camera.film = &film;
	camera.Init();
	Sampler sampler;
	Scene scene;

	//LightTracing lt(&sampler, &camera, 8, 32);
	//lt.Render(scene, camera);

	BidirectionalPathTracing bpt(&sampler, 32, 30, false, false);
	bpt.Render(scene, camera);

	//PathTracing pt(&sampler, 32, 10);
	//pt.Render(scene, camera);

}


int main() {
	
	SceneOne();
	//SceneTwo();
	
	return 0;
}