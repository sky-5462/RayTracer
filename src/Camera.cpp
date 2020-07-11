#include "RayTracer/Camera.h"
#include "Eigen/Geometry"

Camera::Camera(int width, int height) {
	setCamera(Eigen::Vector3f::Zero(), Eigen::Vector3f(0.0f, 0.0f, -1.0f), 50.0f, 0.0f, width, height);
}

void Camera::setCamera(const Eigen::Vector3f& origin, const Eigen::Vector3f& viewPoint,
					   float focal, float rotateAngle, int width, int height) {
	float fWidth = width;
	float fHeight = height;
	this->origin = origin;
	Eigen::Vector3f direction = (viewPoint - origin).normalized();
	Eigen::Vector3f right = direction.cross(Eigen::Vector3f(0.0f, 1.0f, 0.0f)).normalized();
	Eigen::Vector3f down = direction.cross(right).normalized();
	
	// use a 36x24mm frame
	Eigen::Vector3f upHalfLength, rightHalfLength;
	if (fWidth / fHeight >= 1.5f) {
		rightHalfLength = right * (18.0f / focal);
		upHalfLength = down * (18.0f / focal / (fWidth / fHeight));
	}
	else {
		upHalfLength = down * (12.0f / focal);
		rightHalfLength = right * (12.0f / focal * (fWidth / fHeight));
	}
	leftUpCorner = direction - rightHalfLength - upHalfLength;
	rightStep = rightHalfLength / (fWidth / 2.0f);
	downStep = upHalfLength / (fHeight / 2.0f);

	auto rotateMatrix = Eigen::AngleAxisf((3.1415926f / 180.0f) * rotateAngle, direction).toRotationMatrix();
	leftUpCorner = rotateMatrix * leftUpCorner;
	rightStep = rotateMatrix * rightStep;
	downStep = rotateMatrix * downStep;
}

// SSAA 4x sampling
// 1  2
// 3  4
static constexpr std::array<float, 4> xOffset = { -0.25f, 0.25f, -0.25f, 0.25f };
static constexpr std::array<float, 4> yOffset = { -0.25f, -0.25f, 0.25f, 0.25f };

std::array<Ray, 4> Camera::getRay(int x, int y) const {
	std::array<Ray, 4> result;
	for (int i = 0; i < 4; ++i) {
		result[i].origin = origin;
		float xTemp = x + xOffset[i];
		float yTemp = y + yOffset[i];
		result[i].direction = (leftUpCorner + xTemp * rightStep + yTemp * downStep).normalized();
	}
	return result;
}