#include "RayTracer/Camera.h"
#include "Eigen/Geometry"

void Camera::setCamera(const Eigen::Vector4f& origin, const Eigen::Vector4f& viewPoint,
					   float focal, float rotateAngle, int width, int height) {
	float fWidth = width;
	float fHeight = height;
	this->origin = origin;
	Eigen::Vector4f direction = (viewPoint - origin).normalized();
	Eigen::Vector4f right = direction.cross3(Eigen::Vector4f::UnitY()).normalized();
	Eigen::Vector4f down = direction.cross3(right).normalized();
	
	// use a 36x24mm frame
	Eigen::Vector4f upHalfLength, rightHalfLength;
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

	Eigen::Matrix4f rotateMatrix = Eigen::Matrix4f::Zero();
	rotateMatrix.block<3, 3>(0, 0) = Eigen::AngleAxisf((3.1415926f / 180.0f) * rotateAngle, direction.head<3>()).toRotationMatrix();
	leftUpCorner = rotateMatrix * leftUpCorner;
	rightStep = rotateMatrix * rightStep;
	downStep = rotateMatrix * downStep;
}

// SSAA 4x sampling
// 1  2
// 3  4
constexpr std::array<float, 4> xOffset = { -0.25f, 0.25f, -0.25f, 0.25f };
constexpr std::array<float, 4> yOffset = { -0.25f, -0.25f, 0.25f, 0.25f };

std::array<Ray, 4> Camera::getRay(int x, int y) const {
	std::array<Ray, 4> result;
	for (int i = 0; i < 4; ++i) {
		result[i].origin = origin;
		float xTemp = x + xOffset[i];
		float yTemp = y + yOffset[i];
		result[i].direction = leftUpCorner + xTemp * rightStep + yTemp * downStep;
	}
	return result;
}