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

	std::string filename = "Result/Image2.png";
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

	BidirectionalPathTracing bpt(&sampler, 15, 32, false, false);
	bpt.Render(scene, camera);

	//PathTracing pt(&sampler, 12, 128);
	//pt.Render(scene, camera);
}


int main() {
	
	SceneOne();

	
	return 0;
}