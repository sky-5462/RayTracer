#include "RayTracer/Camera.h"

Camera::Camera(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction, const Eigen::Vector3f& up,
			   float vFov, int width, int height) : origin(origin) {
	//test
	rightStep = Eigen::Vector3f(4, 0, 0) / width;
	upStep = Eigen::Vector3f(0, 2, 0) / height;
	leftDownCorner = Eigen::Vector3f(-2, -1, -1);
}

// SSAA 4x sampling
// 1  2
// 3  4
static std::array<float, 4> xOffset = { -0.25f, 0.25f, -0.25f, 0.25f };
static std::array<float, 4> yOffset = { 0.25f, 0.25f, -0.25f, -0.25f };

std::array<Ray, 4> Camera::getRay(int x, int y) const {
	std::array<Ray, 4> result;
	for (int i = 0; i < 4; ++i) {
		result[i].origin = origin;
		auto xTemp = x + xOffset[i];
		auto yTemp = y + yOffset[i];
		result[i].direction = (leftDownCorner + xTemp * rightStep + yTemp * upStep).normalized();
	}
	return result;
}