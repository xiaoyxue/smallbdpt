#include "Light.h"
#include "Intersection.h"

Vec3 AreaLight::Sample(Intersection* lightPoint, double* pdf, const Vec2& u) const
{
	*lightPoint = mpShape->Sample(pdf, u);
	return Le;
}

void AreaLight::SampleOnLight(Intersection* lightPoint, Vec3* dir, double* pdfPos, double* pdfDir, const Vec2& u, const Vec2& v) const
{
	*lightPoint = mpShape->Sample(pdfPos, u);
	Vec3 ss, ts;
	CoordinateSystem(lightPoint->Normal, &ss, &ts);
	Vec3 dirLocal = CosineSampleHemisphere(v);
	double cosTheta = dirLocal.z;
	*dir = (ss * dirLocal.x + ts * dirLocal.y + lightPoint->Normal * dirLocal.z).norm();
	*pdfDir = CosineHemispherePdf(cosTheta);
}
