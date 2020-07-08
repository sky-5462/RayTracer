#include <RayTracer/Triangle.h>
#include <Eigen/QR>
#include <random>

std::pair<float, Eigen::Vector3f> Triangle::hit(const Ray& r) const {
	// get the hit point in that plane
	float temp = planeNormal.dot(vertexPosition(0) - r.origin);
	float t = temp / planeNormal.dot(r.direction);
	// negative t means go backwards and 0 means hit the origin itselt, need to exclude them
	// produce NaN if the ray is in that plane, but still get a false
	if (t > 0.001f) {
		Eigen::Matrix<float, 3, 2> matrix;
		matrix.col(0) = vertexPosition(0) - vertexPosition(2);
		matrix.col(1) = vertexPosition(1) - vertexPosition(2);
		Eigen::Vector3f hitPoint = r.origin + t * r.direction;
		Eigen::Vector2f x = matrix.householderQr().solve(hitPoint - vertexPosition(2));
		float alpha = x(0);
		float beta = x(1);
		if (0.0f <= alpha && alpha <= 1.0f && 0.0f <= beta && beta <= 1.0f && alpha + beta <= 1.0f) {
			Eigen::Vector3f normal = alpha * vertexNormal(0) + beta * vertexNormal(1) +
				(1.0f - (alpha + beta)) * vertexNormal(2);
			return std::make_pair(t, normal.normalized());
		}
		else
			return std::make_pair(FLT_MAX, Eigen::Vector3f());
	}
	else
		return std::make_pair(FLT_MAX, Eigen::Vector3f());
}

std::vector<Eigen::Vector3f> Triangle::diffuse(const Eigen::Vector3f& normal, const Ray& r, const Eigen::Vector3f& hitPoint, int rayNum) const {
	thread_local static std::mt19937 rand;
	thread_local static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

	// let the normal point to the same side with ray
	Eigen::Vector3f tempNormal = (r.direction.dot(normal)) < 0.0f ? normal : -normal;
	Eigen::Vector3f sphereCenter = hitPoint + tempNormal;
	Eigen::Vector3f direction;
	std::vector<Eigen::Vector3f> result;
	result.reserve(rayNum);

	for (int i = 0; i < rayNum; ++i) {
		do {
			direction = sphereCenter;
			direction.x() += dist(rand);
			direction.y() += dist(rand);
			direction.z() += dist(rand);
		} while (direction.squaredNorm() < 1.0f);

		direction.normalize();
		result.push_back(direction);
	}

	return result;
}

Eigen::Vector3f Triangle::specular(const Eigen::Vector3f& normal, const Ray& r) const {
	// output direction must be a unit vector
	float normalProjection = r.direction.dot(normal);
	return r.direction - (2.0f * normalProjection) * normal;
}

// refer to: https://graphicscompendium.com/raytracing/10-reflection-refraction
std::pair<float, Eigen::Vector3f> Triangle::refract(const Eigen::Vector3f& normal, const Ray& r) const {
	float dot = r.direction.dot(normal);

	// outer index / inner index
	float indexRatio = dot > 0.0f ? refractiveIndex : (1.0f / refractiveIndex);

	// turn the normal to the same side with the input ray
	Eigen::Vector3f tempNormal = dot > 0.0f ? -normal : normal;

	// angle with the input ray and the normal
	float cosineTheta = fabsf(dot);

	// determine if we have no refraction (total reflection)
	float squareSine1 = 1.0f - cosineTheta * cosineTheta;
	float squareSine2 = (indexRatio * indexRatio) * squareSine1;
	if (squareSine2 > 1.0f)
		return std::make_pair(0.0f, Eigen::Vector3f());

	// calculate the output ray
	float cosine2 = sqrtf(1.0f - squareSine2);
	Eigen::Vector3f outRay = indexRatio * (r.direction + cosineTheta * tempNormal) - tempNormal * cosine2;

	// determine the proportion of refraction
	// use Schlick's approximation
	float temp = (1.0f - indexRatio) / (1.0f + indexRatio);
	float r0 = temp * temp;
	float reflectProportion = r0 + (1.0f - r0) * powf((1.0f - cosineTheta), 5.0f);

	//return std::make_pair(0.9f, Ray(hitPoint, outRay));
	return std::make_pair(1.0f - reflectProportion, outRay);
}
