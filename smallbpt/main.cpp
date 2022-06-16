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

int main() {
	
	int width = 1024, heigh = 768;
#ifdef _DEBUG
	//width = 40, heigh = 10;
#endif
	std::string filename = "Image21.png";
	Film film(width, heigh, filename);
	Camera camera;
	Vec3 camPos(50, 52, 295.6), d(0, -0.042612, -1);
	d.norm();
	Vec3 u = Vec3(width * .5135 / heigh).norm();
	Vec3 v = (u % d).norm();
	std::cout << camPos + d << std::endl << v << std::endl;
	camera.SetCamera(camPos, d, u, v, 1.0, 28.7993);
	camera.film = &film;
	camera.Init();
	Sampler sampler;
	//LightTracing lt(&sampler, &camera, 8, 32);
	//lt.Render();

	//BidirectionalPathTracing bpt(&sampler, &camera, &film, 16, 32, false, false);
	//bpt.Render();

	PathTracing pt(&sampler, &camera, 12, 128);
	//pt.Render();

	
	return 0;
}