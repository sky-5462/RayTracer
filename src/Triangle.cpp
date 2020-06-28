#include <RayTracer/Triangle.h>

bool Triangle::hit(const Ray& r) const {
	// test
	Eigen::Vector3f center(0, 0, -3);
	float radius = 0.7f;
	auto oc = r.origin - center;
	auto a = r.direction.dot(r.direction);
	auto b = oc.dot(r.direction);
	auto c = oc.dot(oc) - radius * radius;
	float discriminant = b * b - a * c;  // eliminate "4" since it won't affect pos&neg
	if (discriminant > 0) {
		return true;
	}
	return false;

}