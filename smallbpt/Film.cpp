#include "Film.h"
#include "Smallbpt.h"
#include <string>
#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

void Film::AddSplat(Vec3 pRaster, Vec3 Value) {
	int pixelX = (int)(pRaster.x);
	int pixelY = (int)(pRaster.y);
	Spinlocks[pixelY * resX + pixelX].lock();
	Splat[pixelY * resX + pixelX] = Splat[pixelY * resX + pixelX] + Value;
	Spinlocks[pixelY * resX + pixelX].unlock();
}


void Film::AddSample(Vec3 pRaster, Vec3 Value) {
	int pixelX = (int)(pRaster.x);
	int pixelY = (int)(pRaster.y);
	Image[pixelY * resX + pixelX] = Value;
}

void Film::AddVisualPlane(Vec3 pRaster, bool inscreen) {
	int radius = 1;
	int pixelXmin = std::max(0, (int)(pRaster.x) - radius);
	int pixelXmax = std::min(resX - 1, (int)(pRaster.x) + radius);
	int pixelYmin = std::max(0, (int)(pRaster.y) - radius);
	int pixelYmax = std::min(resY - 1, (int)(pRaster.y) + radius);
	for (int pixelX = pixelXmin; pixelX <= pixelXmax; pixelX++) {
		for (int pixelY = pixelYmin; pixelY <= pixelYmax; pixelY++) {
			if(inscreen) VisualPlane[pixelY * resX + pixelX] = Vec3(1, 1, 0);
			else VisualPlane[pixelY * resX + pixelX] = Vec3(1, 0, 0);
		}
	}
}


void Film::AddVisualPlane2(Vec3 pRaster, Vec3 value) {
	int radius = 1;
	int pixelXmin = std::max(0, (int)(pRaster.x) - radius);
	int pixelXmax = std::min(resX - 1, (int)(pRaster.x) + radius);
	int pixelYmin = std::max(0, (int)(pRaster.y) - radius);
	int pixelYmax = std::min(resY - 1, (int)(pRaster.y) + radius);
	for (int pixelX = pixelXmin; pixelX <= pixelXmax; pixelX++) {
		for (int pixelY = pixelYmin; pixelY <= pixelYmax; pixelY++) {
			VisualPlane[pixelY * resX + pixelX] = value;
		}
	}
}

void Film::WriteToVisualImage(std::string visual_filename) {
	for (int j = 0; j < resY; ++j) {
		for (int i = 0; i < resX; ++i) {
			int index_rgb = j * resX * 3 + i * 3;
			int index = j * resX + i;
			if (VisualPlane[index] == Vec3(0, 0, 0)) continue;
			RGBs[index_rgb] = (unsigned char)(toInt(VisualPlane[index].x));
			RGBs[index_rgb + 1] = (unsigned char)(toInt(VisualPlane[index].y));
			RGBs[index_rgb + 2] = (unsigned char)(toInt(VisualPlane[index].z));
		}
	}
	//std::string visual_filename = "visual_image.png";
	FILE *fp = fopen(visual_filename.c_str(), "wb");
	//Wsvpng(fp, resX, resY, RGBs, 0);

	fclose(fp);
}

void Film::WriteToImage() {
	static const int channels = 3;
	for (int j = 0; j < resY; ++j) {
		for (int i = 0; i < resX; ++i) {
			int index = j * resX + i;
			Image[index] = Image[index] + Splat[index];
			//std::cout << Image[index].x << " " << Image[index].y << " " << Image[index].z << std::endl;
		}
	}

	for (int j = 0; j < resY; ++j) {
		for (int i = 0; i < resX; ++i) {
			int index_rgb = j * resX * 3 + i * 3;
			int index = j * resX + i;
			RGBs[index_rgb] = (unsigned char)(toInt(Image[index].x));
			RGBs[index_rgb + 1] = (unsigned char)(toInt(Image[index].y));
			RGBs[index_rgb + 2] = (unsigned char)(toInt(Image[index].z));
		}
	}

	stbi_write_png(filename.c_str(), resX, resY, channels, &RGBs[0], channels * resX);
	//FILE *fp = fopen(filename.c_str(), "wb");
	//svpng(fp, resX, resY, RGBs, 0);

	//fclose(fp);
}