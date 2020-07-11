#pragma once

#include <Eigen/Core>

struct Ray {
	Eigen::Vector4f origin;
	Eigen::Vector4f direction;

	Ray() {}
	Ray(const Eigen::Vector4f& origin, const Eigen::Vector4f& direction) :
		origin(origin), direction(direction) { }
};