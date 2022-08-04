#include "Geometry.h"
#include "Intersection.h"

bool Triangle::intersect(const Ray& ray, Intersection* isect, double* t) {
	Vec3 s1 = cross(ray.d, e2);
	double divisor = dot(s1, e1);

	if (divisor == 0.)
		return false;
	double invDivisor = 1.f / divisor;

	// Compute first barycentric coordinate
	Vec3 s = ray.o - p0;
	double b1 = dot(s, s1) * invDivisor;
	if (b1 < 0. || b1 > 1.)
		return false;

	// Compute second barycentric coordinate
	Vec3 s2 = cross(s, e1);
	double b2 = dot(ray.d, s2) * invDivisor;
	if (b2 < 0. || b1 + b2 > 1.)
		return false;

	// Compute _t_ to intersection point
	double tHit = dot(e2, s2) * invDivisor;
	if (tHit < ray.tmin || tHit > ray.tmax)
		return false;

	*t = tHit;
	isect->HitPoint = (1 - b1 - b2) * p0 + b1 * p1 + b2 * p2;
	isect->b1 = b1;
	isect->b2 = b2;

	return true;
}
Intersection Triangle::Sample(double* pdf, const Vec3& u) const
{
	Vec3 b = UniformSampleTriangle(u);
	Intersection isect;
	isect.HitPoint = b[0] * p0 + b[1] * p1 + (1 - b[0] - b[1]) * p2;
	isect.Normal = normal;
	*pdf = 1 / Area();
	return isect;
}

Intersection Sphere::Sample(double* pdf, const Vec3& u) const
{
	*pdf = 1.0 / Area();
	Vec3 samplePoint = UniformSampleSphere(u);
	Vec3 dir = samplePoint - p;
	Vec3 scaledDir = rad * dir;
	Vec3 point = p + scaledDir;
	Intersection isect;
	isect.HitPoint = point;
	isect.Normal = (point - p).norm();
	isect.SurfaceNormal = isect.Normal;
	return isect;
}
