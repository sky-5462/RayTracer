#include <RayTracer/Triangle.h>
#include <Eigen/LU>

bool Triangle::hit(const Ray& r) const {
	// get the hit point in that plane
	auto temp = normal.dot(vertex(0) - r.origin);
	auto t = temp / normal.dot(r.direction);
	// negative t means go backwards
	// produce NaN if the ray is in that plane, but still get a false
	if (t < 0.0f) 
		return false;

	Eigen::Matrix3f matrix;
	matrix.col(0) = vertex(2) - vertex(0);
	matrix.col(1) = vertex(2) - vertex(1);
	matrix.col(2) = r.direction;
	// the matrix must be invertible
	Eigen::Vector3f x = matrix.partialPivLu().solve(vertex(2) - r.origin);
	float alpha = x(0);
	float beta = x(1);
	if (0.0f <= alpha && alpha <= 1.0f && 0.0f <= beta && beta <= 1.0f && alpha + beta <= 1.0f)
		return true;
	else
		return false;
}