#include <iostream>
#include "Scene.h"
#include "Camera.h"
#include "Geometry.h"
#include "Film.h"
#include "Sampler.h"
#include "Integrator.h"
#include "LightTracing.h"
#include "PathTracing.h"
#include "BDPT.h"


void SceneOne() {
	int width = 1024, height = 768;

	std::string filename = "Result/Image8.png";
	Film film(width, height, filename);
	Camera camera;
	Vec3 camPos(50, 52, 295.6), d(0, -0.042612, -1);
	d.norm();
	Vec3 u = Vec3(width * .5135 / height).norm();
	Vec3 v = (u % d).norm();
	std::cout << camPos + d << std::endl << v << std::endl;
	camera.SetCamera(camPos, d, u, v, 1.0, 28.7993);
	camera.film = &film;
	camera.Init();
	Sampler sampler;
	Scene scene;

	//LightTracing lt(&sampler, &camera, 8, 32);
	//lt.Render(scene, camera);

	BidirectionalPathTracing bpt(&sampler, 16, 32, false, false);
	bpt.Render(scene, camera);

	//PathTracing pt(&sampler, 12, 128);
	//pt.Render(scene, camera);
}

void SceneTwo() {
	int resX = 1024, resY = 1024;
	std::string filename = "Result/Image9.png";
	Film film(resX, resY, filename);
	Camera camera;
	Vec3 camPos(0, 0, 3);
	Vec3 d = Vec3(0, 0, -1);
	Vec3 v(0, 1, 0);
	Vec3 u(1, 0, 0);
	double fovy = 53.13010235415597f;
	camera.SetCamera(camPos, d, u, v, 1, fovy);
	camera.film = &film;
	camera.Init();
	Sampler sampler;
	Scene scene;

	BidirectionalPathTracing bpt(&sampler, 16, 32, false, false);
	bpt.Render(scene, camera);
}

int main() {
	
	SceneTwo();
	
	return 0;
}