#include "TiledIntegrator.h"
#include "Film.h"
#include "Camera.h"

void TiledIntegrator::Render(const Scene& scene, const Camera& camera) {

	fprintf(stderr, "Rendering ...\n");

	Film* pFilm = camera.GetFilm();
	int resX = pFilm->resX, resY = pFilm->resY;
	const int tileSize = 32;
	std::vector<Tile> tiles;
	int tileIndex = 0;
	for (int y = 0; y < resY; y += tileSize) {
		for (int x = 0; x < resX; x += tileSize) {
			Tile tile;
			tile.minX = x;
			tile.minY = y;
			tile.maxX = std::min(resX, x + tileSize);
			tile.maxY = std::min(resY, y + tileSize);
			tiles.push_back(tile);
		}
	}
	std::atomic<int64_t> workDone = 0;
	const int tileCount = tiles.size();
#pragma omp parallel for schedule(dynamic, 1)
	for(int i = 0; i < tileCount; i++){
		const Tile& tile = tiles[i];
		for (int y = tile.minY; y < tile.maxY; ++y) {
			for (int x = tile.minX; x < tile.maxX; ++x) {
				for (int spp = 0; spp < mSpp; ++spp) {
					
					Vec2 pixelSample = mpSampler->Get2D();
					Ray ray = camera.GenerateRay(x, y, pixelSample, 0);
					Vec3 L = Li(scene, ray);
					pFilm->AddSample(Vec3(x + pixelSample.x, y + pixelSample.y, 0), L );
				}
			}
		}
		workDone += 1;
		double percentage = 100.f * workDone / tileCount;
		fprintf(stderr, "\rPercentage: %5.2f%%", percentage);
	}
	pFilm->WriteToImage();
}