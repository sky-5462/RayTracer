#pragma once

#include "RayTracer/Ray.h"
#include <Eigen/Core>
#include <array>

class Camera {
public:
	Camera(int width, int height);
	void setCamera(const Eigen::Vector3f& origin, const Eigen::Vector3f& viewPoint,
				   float focal, int width, int height);

	std::array<Ray, 4> getRay(int x, int y) const;

private:
	Eigen::Vector3f origin;
	Eigen::Vector3f rightStep;
	Eigen::Vector3f downStep;
	Eigen::Vector3f leftUpCorner;
};