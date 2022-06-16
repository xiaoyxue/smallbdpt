#ifndef FILM_H
#define FILM_H
#include "Geometry.h"
#include <vector>
#include "Threading.h"
class Film {
public:
	Film(int w = 1024, int h = 768, std::string  _filename = "Image.png") : width(w), heigh(h), filename(_filename){
		Image = new Vec3[width * heigh]();
		Splat = new Vec3[width * heigh]();
		RGBs = new unsigned char[width * heigh * 3]();
		aspect = (double)(width) / (double)(heigh);

		Spinlocks.resize(width * heigh);

		VisualPlane = new Vec3[width * heigh]();

	}
	~Film() {
		delete[] Image;
		delete[] Splat;
		delete[] RGBs;
		delete[] VisualPlane;
	}
	void AddSplat(Vec3 pRaster, Vec3 Value);
	void AddSample(Vec3 pRaster, Vec3 Value);
	void AddVisualPlane(Vec3 pRater, bool inscreen = false);
	void AddVisualPlane2(Vec3 pRater, Vec3 value);
	void WriteToVisualImage(std::string visual_filename = "visual_image.png");
	void WriteToImage();
public:
	Vec3 LL, LU, RL, RU;
	double aspect;
	double Area;
	int width, heigh;

private:
	Vec3 *Image, *Splat;
	unsigned char *RGBs;
	std::string filename;
	Vec3 *VisualPlane;
	std::vector<Spinlock> Spinlocks;
};


#endif