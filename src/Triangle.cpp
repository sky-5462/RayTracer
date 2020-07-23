#include <RayTracer/Triangle.h>
#include <random>
#include <cfloat>
#include <array>

constexpr unsigned randMask = 0x1FF;
constexpr unsigned precision = randMask + 1;
const std::array<float, precision> cosTable = []() {
	std::array<float, precision> temp;
	for (int i = 0; i < precision; ++i) {
		temp[i] = cosf(i * 3.1415926f * 2.0f / precision);
	}
	return temp;
}();
const std::array<float, precision> sinTable = []() {
	std::array<float, precision> temp;
	for (int i = 0; i < precision; ++i) {
		temp[i] = sinf(i * 3.1415926f * 2.0f / precision);
	}
	return temp;
}();

Eigen::Vector4f Triangle::hit(const Ray& r) const {
	// 求光线与三角形所在平面相交时的ｔ值
	// 光线前进相交于平面时ｔ必然为正数，这里的0.001排除误差判定为光线与射出点所在平面本身相交
	// 光线平行于平面会产生NaN，也能排除掉
	float temp = planeNormal.dot(vertexPosition(0) - r.origin);
	float t = temp / planeNormal.dot(r.direction);
	if (t > 0.001f) {
		Eigen::Vector4f hitPoint = r.origin + t * r.direction;
		Eigen::Vector4f a1 = vertexPosition(0) - vertexPosition(2);
		Eigen::Vector4f a2 = vertexPosition(1) - vertexPosition(2);
		Eigen::Vector4f b = hitPoint - vertexPosition(2);

		// 完全主元法高斯消元
		// 增广矩阵各列
		auto matrixCol1 = _mm_load_ps(a1.data());
		auto matrixCol2 = _mm_load_ps(a2.data());
		auto matrixCol3 = _mm_load_ps(b.data());
		// 矩阵转置，便于向量化操作
		auto temp1 = _mm_unpacklo_ps(matrixCol1, matrixCol2);
		auto temp2 = _mm_unpackhi_ps(matrixCol1, matrixCol2);
		auto temp3 = _mm_unpacklo_ps(matrixCol3, matrixCol3);
		auto temp4 = _mm_unpackhi_ps(matrixCol3, matrixCol3);
		std::array<__m128, 3> matrixRow;
		matrixRow[0] = _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(temp1), _mm_castps_pd(temp3)));
		matrixRow[1] = _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(temp1), _mm_castps_pd(temp3)));
		matrixRow[2] = _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(temp2), _mm_castps_pd(temp4)));
		// 选取第一个主元，主元列绝对值最大的一行
		int index = 0;
		float max = std::fabsf(matrixRow[0].m128_f32[0]);
		if (std::fabsf(matrixRow[1].m128_f32[0]) > max) {
			max = std::fabsf(matrixRow[1].m128_f32[0]);
			index = 1;
		}
		if (std::fabsf(matrixRow[2].m128_f32[0]) > max) {
			index = 2;
		}
		std::swap(matrixRow[0], matrixRow[index]);
		// 第一行主元归一化
		auto divisor = _mm_broadcast_ss(matrixRow[0].m128_f32);
		matrixRow[0] = _mm_div_ps(matrixRow[0], divisor);
		// 消去后两行的第一列
		for (int i = 1; i < 3; ++i) {
			auto multiplier = _mm_broadcast_ss(matrixRow[i].m128_f32);
			matrixRow[i] = _mm_fnmadd_ps(matrixRow[0], multiplier, matrixRow[i]);
		}
		// 选取第二个主元
		index = 1;
		if (std::fabsf(matrixRow[2].m128_f32[1]) > std::fabsf(matrixRow[1].m128_f32[1])) {
			index = 2;
		}
		std::swap(matrixRow[1], matrixRow[index]);
		// 第二行主元归一化
		divisor = _mm_broadcast_ss(&matrixRow[1].m128_f32[1]);
		matrixRow[1] = _mm_div_ps(matrixRow[1], divisor);
		// 不用管第三行，消去第一行的第二列
		auto multiplier = _mm_broadcast_ss(&matrixRow[0].m128_f32[1]);
		matrixRow[0] = _mm_fnmadd_ps(matrixRow[1], multiplier, matrixRow[0]);

		// 得到结果
		float alpha = matrixRow[0].m128_f32[2];
		float beta = matrixRow[1].m128_f32[2];
		bool accept = (0.0f <= alpha) & (alpha <= 1.0f) & (0.0f <= beta) & (beta <= 1.0f) & (alpha + beta <= 1.0f);
		if (accept) {
			return Eigen::Vector4f(alpha, beta, t, t);
		}
		else
			return Eigen::Vector4f::Constant(FLT_MAX);
	}
	else
		return Eigen::Vector4f::Constant(FLT_MAX);
}

std::vector<Eigen::Vector4f> Triangle::diffuse(const Eigen::Vector4f& normal, const Ray& r, int diffuseRayNum) const {
	thread_local static std::mt19937 rand;

	// 法向量指向光线射入的方向
	Eigen::Vector4f tempNormal = (r.direction.dot(normal)) < 0.0f ? normal : -normal;
	std::vector<Eigen::Vector4f> result(diffuseRayNum);

	for (int i = 0; i < diffuseRayNum; ++i) {
		// 单位球内的均匀分布

		// 水平面随机弧度, [0, 2 * PI)
		float phiUnit = rand() & randMask;
		Eigen::Vector4f horizontal(cosTable[phiUnit], sinTable[phiUnit], 0.0f, 0.0f);

		// 垂直方向随机弧度, [0, PI)
		float thetaUnit = (rand() & randMask) >> 1;

		// 最终随机方向
		Eigen::Vector4f offset = horizontal * sinTable[thetaUnit] + cosTable[thetaUnit] * Eigen::Vector4f::UnitZ();

		result[i] = tempNormal + offset;
	}

	return result;
}

std::vector<Eigen::Vector4f> Triangle::specular(const Eigen::Vector4f& normal, const Ray& r, int specularRayNum) const {
	thread_local static std::mt19937 rand;

	float normalProjection = r.direction.dot(normal);
	Eigen::Vector4f direction = r.direction - (2.0f * normalProjection) * normal;

	std::vector<Eigen::Vector4f> result(specularRayNum);

	for (int i = 0; i < specularRayNum; ++i) {
		// 单位球内的均匀分布

		// 水平面随机弧度, [0, 2 * PI)
		float phiUnit = rand() & randMask;
		Eigen::Vector4f horizontal(cosTable[phiUnit], sinTable[phiUnit], 0.0f, 0.0f);

		// 垂直方向随机弧度, [0, PI)
		float thetaUnit = (rand() & randMask) >> 1;

		// 最终随机方向
		Eigen::Vector4f offset = horizontal * sinTable[thetaUnit] + cosTable[thetaUnit] * Eigen::Vector4f::UnitZ();

		result[i] = direction + offset * specularRoughness;
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
