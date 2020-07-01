#include <RayTracer/Triangle.h>
#include <Eigen/LU>
#include <random>
#include <ctime>

#define DIFFUSE_RAY_NUM 10

std::mt19937 Triangle::rand = std::mt19937();

std::pair<bool, float> Triangle::hit(const Ray& r) const {
	// get the hit point in that plane
	auto temp = normal.dot(vertex(0) - r.origin);
	auto t = temp / normal.dot(r.direction);
	// negative t means go backwards
	// produce NaN if the ray is in that plane, but still get a false
	if (t <= 0.0f)
		return std::make_pair(false, 0.0f);

	Eigen::Matrix3f matrix;
	matrix.col(0) = vertex(2) - vertex(0);
	matrix.col(1) = vertex(2) - vertex(1);
	matrix.col(2) = r.direction;
	// the matrix must be invertible
	Eigen::Vector3f x = matrix.partialPivLu().solve(vertex(2) - r.origin);
	float alpha = x(0);
	float beta = x(1);
	if (0.0f <= alpha && alpha <= 1.0f && 0.0f <= beta && beta <= 1.0f && alpha + beta <= 1.0f)
		return std::make_pair(true, x(2));
	else
		return std::make_pair(false, 0.0f);
}

std::vector<Ray> Triangle::diffuse(const Ray& r, const Eigen::Vector3f& hitPoint) const {
	// let the normal point to the same side with ray
	Eigen::Vector3f tempNormal = (r.direction.dot(normal)) < 0.0f ? normal : -normal;

	std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
	Eigen::Vector3f sphereCenter = hitPoint + tempNormal;
	Eigen::Vector3f direction;
	std::vector<Ray> result;
	result.reserve(DIFFUSE_RAY_NUM);

	for (int i = 0; i < DIFFUSE_RAY_NUM; ++i) {
		do {
			direction = sphereCenter;
			direction.x() += dist(Triangle::rand);
			direction.y() += dist(Triangle::rand);
			direction.z() += dist(Triangle::rand);
		} while (direction.squaredNorm() < 1.0f);

		direction.normalize();
		result.push_back(Ray(hitPoint, direction));
	}

	return result;
}

Ray Triangle::specular(const Ray& r, const Eigen::Vector3f& hitPoint) const {
	// output direction must be a unit vector
	auto normalProjection = r.direction.dot(normal);
	Eigen::Vector3f out = r.direction - (2.0f * normalProjection) * normal;
	return Ray(hitPoint, out);
}

// refer to: https://graphicscompendium.com/raytracing/10-reflection-refraction
std::pair<float, Ray> Triangle::refract(const Ray& r, const Eigen::Vector3f& hitPoint) const {
	auto dot = r.direction.dot(normal);
	auto indexRatio = dot > 0.0f ? refractiveIndex : (1.0f / refractiveIndex);
	Eigen::Vector3f tempNormal = dot > 0.0f ? -normal : normal;
	auto cosineTheta = fabsf(dot);

	// determine if we have no refraction (total reflection)
	auto squareSine1 = 1.0f - cosineTheta * cosineTheta;
	auto squareSine2 = indexRatio * indexRatio * squareSine1;
	if (squareSine2 > 1.0f)
		return std::make_pair(0.0f, Ray());

	// calculate the output ray
	auto cosine2 = sqrtf(1.0f - squareSine2);
	Eigen::Vector3f outRay = indexRatio * (r.direction + cosineTheta * tempNormal) - tempNormal * cosine2;

	// determine the proportion of refraction
	// use Schlick's approximation
	auto temp = (1.0f - indexRatio) / (1.0f + indexRatio);
	auto r0 = temp * temp;
	auto reflectProportion = r0 + (1.0f - r0) * powf((1.0f - cosineTheta), 5.0f);

	//return std::make_pair(0.9f, Ray(hitPoint, outRay));
	return std::make_pair(1.0f - reflectProportion, Ray(hitPoint, outRay));
}
