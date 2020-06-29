#include <RayTracer/Triangle.h>
#include <Eigen/LU>
#include <random>
#include <ctime>

std::mt19937 Triangle::rand = std::mt19937();

std::tuple<bool, float> Triangle::hit(const Ray& r) const {
	// get the hit point in that plane
	auto temp = normal.dot(vertex(0) - r.origin);
	auto t = temp / normal.dot(r.direction);
	// negative t means go backwards
	// produce NaN if the ray is in that plane, but still get a false
	if (t <= 0.0f)
		return std::make_tuple(false, 0.0f);

	Eigen::Matrix3f matrix;
	matrix.col(0) = vertex(2) - vertex(0);
	matrix.col(1) = vertex(2) - vertex(1);
	matrix.col(2) = r.direction;
	// the matrix must be invertible
	Eigen::Vector3f x = matrix.partialPivLu().solve(vertex(2) - r.origin);
	float alpha = x(0);
	float beta = x(1);
	if (0.0f <= alpha && alpha <= 1.0f && 0.0f <= beta && beta <= 1.0f && alpha + beta <= 1.0f)
		return std::make_tuple(true, x(2));
	else
		return std::make_tuple(false, 0.0f);
}

Ray Triangle::diffuse(const Ray& r, const Eigen::Vector3f& hitPoint) const {
	// let the normal point to the same side with ray
	Eigen::Vector3f tempNormal = (r.direction.dot(normal)) < 0.0f ? normal : -normal;

	std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
	Eigen::Vector3f sphereCenter = hitPoint + normal;
	Eigen::Vector3f direction = sphereCenter;
	do {
		direction = sphereCenter;
		direction.x() += dist(Triangle::rand);
		direction.y() += dist(Triangle::rand);
		direction.z() += dist(Triangle::rand);
	} while (direction.squaredNorm() < 1.0f);

	return Ray(hitPoint, direction);
}