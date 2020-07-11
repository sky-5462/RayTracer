#include <RayTracer/Triangle.h>
#include <Eigen/QR>
#include <random>

std::tuple<float, float, float> Triangle::hit(const Ray& r) const {
	// get the hit point in that plane
	float temp = planeNormal.dot(vertexPosition(0) - r.origin);
	float t = temp / planeNormal.dot(r.direction);
	// negative t means go backwards and 0 means hit the origin itselt, need to exclude them
	// produce NaN if the ray is in that plane, but still get a false
	if (t > 0.001f) {
		Eigen::Matrix<float, 3, 2> matrix;
		matrix.col(0) = (vertexPosition(0) - vertexPosition(2)).head<3>();
		matrix.col(1) = (vertexPosition(1) - vertexPosition(2)).head<3>();
		Eigen::Vector4f hitPoint = r.origin + t * r.direction;
		Eigen::Vector2f x = matrix.householderQr().solve((hitPoint - vertexPosition(2)).head<3>());
		float alpha = x(0);
		float beta = x(1);
		if (0.0f <= alpha && alpha <= 1.0f && 0.0f <= beta && beta <= 1.0f && alpha + beta <= 1.0f) {
			return std::make_tuple(t, alpha, beta);
		}
		else
			return std::make_tuple(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	else
		return std::make_tuple(FLT_MAX, FLT_MAX, FLT_MAX);
}

std::vector<Eigen::Vector4f> Triangle::diffuse(const Eigen::Vector4f& normal, const Ray& r, int diffuseRayNum) const {
	thread_local static std::mt19937 rand;
	thread_local static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

	// let the normal point to the same side with ray
	Eigen::Vector4f tempNormal = (r.direction.dot(normal)) < 0.0f ? normal : -normal;
	std::vector<Eigen::Vector4f> result(diffuseRayNum);

	for (int i = 0; i < diffuseRayNum; ++i) {
		// uniform distrition in the the sphere
		Eigen::Vector4f offset;
		do {
			offset = Eigen::Vector4f(dist(rand), dist(rand), dist(rand), 0.0f);
		} while (offset.squaredNorm() < 1.0f);

		// uniform distrition in all directions
		offset.normalize();
		result[i] = (tempNormal + offset).normalized();
	}

	return result;
}

std::vector<Eigen::Vector4f> Triangle::specular(const Eigen::Vector4f& normal, const Ray& r, int specularRayNum) const {
	thread_local static std::mt19937 rand;
	thread_local static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

	float normalProjection = r.direction.dot(normal);
	Eigen::Vector4f direction = r.direction - (2.0f * normalProjection) * normal;

	std::vector<Eigen::Vector4f> result(specularRayNum);

	for (int i = 0; i < specularRayNum; ++i) {
		// uniform distrition in the the sphere
		Eigen::Vector4f offset;
		do {
			offset = Eigen::Vector4f(dist(rand), dist(rand), dist(rand), 0.0f);
		} while (offset.squaredNorm() < 1.0f);

		// for specular ray, we can use a Gaussian-like distribution
		// output ray may go below the surface returning black (non-transparent)
		// or refractive color (transparent)
		result[i] = (direction + offset * specularRoughness).normalized();
	}

	return result;
}

// refer to: https://graphicscompendium.com/raytracing/10-reflection-refraction
std::pair<float, Eigen::Vector4f> Triangle::refract(const Eigen::Vector4f& normal, const Ray& r) const {
	float dot = r.direction.dot(normal);

	// outer index / inner index
	float indexRatio = dot > 0.0f ? refractiveIndex : (1.0f / refractiveIndex);

	// turn the normal to the same side with the input ray
	Eigen::Vector4f tempNormal = dot > 0.0f ? -normal : normal;

	// angle with the input ray and the normal
	float cosineTheta = fabsf(dot);

	// determine if we have no refraction (total reflection)
	float squareSine1 = 1.0f - cosineTheta * cosineTheta;
	float squareSine2 = (indexRatio * indexRatio) * squareSine1;
	if (squareSine2 > 1.0f)
		return std::make_pair(0.0f, Eigen::Vector4f());

	// calculate the output ray
	float cosine2 = sqrtf(1.0f - squareSine2);
	Eigen::Vector4f outRay = indexRatio * (r.direction + cosineTheta * tempNormal) - tempNormal * cosine2;

	// determine the proportion of refraction
	// use Schlick's approximation
	float temp = (1.0f - indexRatio) / (1.0f + indexRatio);
	float r0 = temp * temp;
	float squareTemp = (1.0f - cosineTheta) * (1.0f - cosineTheta);
	float pow5 = squareTemp * squareTemp * (1.0f - cosineTheta);
	float reflectProportion = r0 + (1.0f - r0) * pow5;

	return std::make_pair(1.0f - reflectProportion, outRay);
}
