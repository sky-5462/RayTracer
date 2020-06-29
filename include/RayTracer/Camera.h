#pragma once

#include "RayTracer/Ray.h"
#include <Eigen/Core>
#include <array>

class Camera {
public:
	Camera(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction, const Eigen::Vector3f& up,
		   float vFov, int width, int height);

	std::array<Ray, 4> getRay(int x, int y) const;

private:
	Eigen::Vector3f origin;
	Eigen::Vector3f rightStep;
	Eigen::Vector3f upStep;
	Eigen::Vector3f leftDownCorner;
};