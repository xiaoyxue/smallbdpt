#include "VisibilityTester.h"
#include "Scene.h"

bool VisibilityTester::Unoccluded(const Scene& scene) const {
	Ray ray = mP0.SpawnTo(mP1);
	return !scene.Intersect(ray);
}
