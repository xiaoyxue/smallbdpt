#include "Film.h"
#include "Smallbpt.h"
#include <string>
#include "svpng.inc"
#include <iostream>
void Film::AddSplat(Vec3 pRaster, Vec3 Value) {
	int pixelX = (int)(pRaster.x);
	int pixelY = (int)(pRaster.y);
	Spinlocks[pixelY * width + pixelX].lock();
	Splat[pixelY * width + pixelX] = Splat[pixelY * width + pixelX] + Value;
	Spinlocks[pixelY * width + pixelX].unlock();
}


void Film::AddSample(Vec3 pRaster, Vec3 Value) {
	int pixelX = (int)(pRaster.x);
	int pixelY = (int)(pRaster.y);
	Image[pixelY * width + pixelX] = Value;
}

void Film::AddVisualPlane(Vec3 pRaster, bool inscreen) {
	int radius = 1;
	int pixelXmin = std::max(0, (int)(pRaster.x) - radius);
	int pixelXmax = std::min(width - 1, (int)(pRaster.x) + radius);
	int pixelYmin = std::max(0, (int)(pRaster.y) - radius);
	int pixelYmax = std::min(heigh - 1, (int)(pRaster.y) + radius);
	for (int pixelX = pixelXmin; pixelX <= pixelXmax; pixelX++) {
		for (int pixelY = pixelYmin; pixelY <= pixelYmax; pixelY++) {
			if(inscreen) VisualPlane[pixelY * width + pixelX] = Vec3(1, 1, 0);
			else VisualPlane[pixelY * width + pixelX] = Vec3(1, 0, 0);
		}
	}
}


void Film::AddVisualPlane2(Vec3 pRaster, Vec3 value) {
	int radius = 1;
	int pixelXmin = std::max(0, (int)(pRaster.x) - radius);
	int pixelXmax = std::min(width - 1, (int)(pRaster.x) + radius);
	int pixelYmin = std::max(0, (int)(pRaster.y) - radius);
	int pixelYmax = std::min(heigh - 1, (int)(pRaster.y) + radius);
	for (int pixelX = pixelXmin; pixelX <= pixelXmax; pixelX++) {
		for (int pixelY = pixelYmin; pixelY <= pixelYmax; pixelY++) {
			VisualPlane[pixelY * width + pixelX] = value;
		}
	}
}

void Film::WriteToVisualImage(std::string visual_filename) {
	for (int j = 0; j < heigh; ++j) {
		for (int i = 0; i < width; ++i) {
			int index_rgb = j * width * 3 + i * 3;
			int index = j * width + i;
			if (VisualPlane[index] == Vec3(0, 0, 0)) continue;
			RGBs[index_rgb] = (unsigned char)(toInt(VisualPlane[index].x));
			RGBs[index_rgb + 1] = (unsigned char)(toInt(VisualPlane[index].y));
			RGBs[index_rgb + 2] = (unsigned char)(toInt(VisualPlane[index].z));
		}
	}
	//std::string visual_filename = "visual_image.png";
	FILE *fp = fopen(visual_filename.c_str(), "wb");
	//Wsvpng(fp, width, heigh, RGBs, 0);

	fclose(fp);
}

void Film::WriteToImage() {
	
	for (int j = 0; j < heigh; ++j) {
		for (int i = 0; i < width; ++i) {
			int index = j * width + i;
			Image[index] = Image[index] + Splat[index];
			//std::cout << Image[index].x << " " << Image[index].y << " " << Image[index].z << std::endl;
		}
	}

	for (int j = 0; j < heigh; ++j) {
		for (int i = 0; i < width; ++i) {
			int index_rgb = j * width * 3 + i * 3;
			int index = j * width + i;
			RGBs[index_rgb] = (unsigned char)(toInt(Image[index].x));
			RGBs[index_rgb + 1] = (unsigned char)(toInt(Image[index].y));
			RGBs[index_rgb + 2] = (unsigned char)(toInt(Image[index].z));
		}
	}

	FILE *fp = fopen(filename.c_str(), "wb");
	svpng(fp, width, heigh, RGBs, 0);

	fclose(fp);
}