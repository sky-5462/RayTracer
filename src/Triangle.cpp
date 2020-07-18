#include <RayTracer/Triangle.h>
#include <Eigen/QR>
#include <random>

std::tuple<float, float, float> Triangle::hit(const Ray& r) const {
	// 求光线与三角形所在平面相交时的ｔ值
	// 光线前进相交于平面时ｔ必然为正数，这里的0.001排除误差判定为光线与射出点所在平面本身相交
	// 光线平行于平面会产生NaN，也能排除掉
	float temp = planeNormal.dot(vertexPosition(0) - r.origin);
	float t = temp / planeNormal.dot(r.direction);
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

	// 法向量指向光线射入的方向
	Eigen::Vector4f tempNormal = (r.direction.dot(normal)) < 0.0f ? normal : -normal;
	std::vector<Eigen::Vector4f> result(diffuseRayNum);

	for (int i = 0; i < diffuseRayNum; ++i) {
		// 单位球内的均匀分布
		Eigen::Vector4f offset;
		do {
			offset = Eigen::Vector4f(dist(rand), dist(rand), dist(rand), 0.0f);
		} while (offset.squaredNorm() < 1.0f);

		// 归一化到球面上，产生各方向的均匀分布
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
		// 单位球内的均匀分布
		Eigen::Vector4f offset;
		do {
			offset = Eigen::Vector4f(dist(rand), dist(rand), dist(rand), 0.0f);
		} while (offset.squaredNorm() < 1.0f);

		// 不进行归一化，各方向类似于高斯分布，对于镜面反射来说可以接受
		result[i] = (direction + offset * specularRoughness).normalized();
	}

	return result;
}

// refer to: https://graphicscompendium.com/raytracing/10-reflection-refraction
std::pair<float, Eigen::Vector4f> Triangle::refract(const Eigen::Vector4f& normal, const Ray& r) const {
	float dot = r.direction.dot(normal);

	// 入射折射率:出射折射率
	float indexRatio = dot > 0.0f ? refractiveIndex : (1.0f / refractiveIndex);

	// 法向量指向光线射入的方向
	Eigen::Vector4f tempNormal = dot > 0.0f ? -normal : normal;

	// 入射光线和法向量之间的夹角的余弦
	float cosineTheta = fabsf(dot);

	// 是否发生全反射
	float squareSine1 = 1.0f - cosineTheta * cosineTheta;
	float squareSine2 = (indexRatio * indexRatio) * squareSine1;
	if (squareSine2 > 1.0f)
		return std::make_pair(0.0f, Eigen::Vector4f());

	// 出射光线
	float cosine2 = sqrtf(1.0f - squareSine2);
	Eigen::Vector4f outRay = indexRatio * (r.direction + cosineTheta * tempNormal) - tempNormal * cosine2;

	// 计算折射的比例，Schlick's approximation
	float temp = (1.0f - indexRatio) / (1.0f + indexRatio);
	float r0 = temp * temp;
	float squareTemp = (1.0f - cosineTheta) * (1.0f - cosineTheta);
	float pow5 = squareTemp * squareTemp * (1.0f - cosineTheta);
	float reflectProportion = r0 + (1.0f - r0) * pow5;

	return std::make_pair(1.0f - reflectProportion, outRay);
}
