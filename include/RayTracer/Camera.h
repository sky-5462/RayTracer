#pragma once

#include "RayTracer/Ray.h"
#include <Eigen/Core>
#include <array>

class Camera {
public:
	Camera(int width, int height);
	void setCamera(const Eigen::Vector4f& origin, const Eigen::Vector4f& viewPoint,
				   float focal, float rotateAngle, int width, int height);

	std::array<Ray, 4> getRay(int x, int y) const;

private:
	Eigen::Vector4f origin;
	Eigen::Vector4f rightStep;
	Eigen::Vector4f downStep;
	Eigen::Vector4f leftUpCorner;
};