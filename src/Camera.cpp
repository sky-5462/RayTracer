#include "RayTracer/Camera.h"

Camera::Camera(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction, const Eigen::Vector3f& up,
			   float vFov, int width, int height) : origin(origin) {
	//test
	rightStep = Eigen::Vector3f(4, 0, 0) / width;
	upStep = Eigen::Vector3f(0, 2, 0) / height;
	leftDownCorner = Eigen::Vector3f(-2, -1, -1);
}

Ray Camera::getRay(int x, int y) const {
	Ray ray;
	ray.origin = origin;
	ray.direction = leftDownCorner + x * rightStep + y * upStep;
	return ray;
}