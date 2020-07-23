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
	// �����������������ƽ���ཻʱ�ģ�ֵ
	// ����ǰ���ཻ��ƽ��ʱ����ȻΪ�����������0.001�ų�����ж�Ϊ���������������ƽ�汾���ཻ
	// ����ƽ����ƽ������NaN��Ҳ���ų���
	float temp = planeNormal.dot(vertexPosition(0) - r.origin);
	float t = temp / planeNormal.dot(r.direction);
	if (t > 0.001f) {
		Eigen::Vector4f hitPoint = r.origin + t * r.direction;
		Eigen::Vector4f a1 = vertexPosition(0) - vertexPosition(2);
		Eigen::Vector4f a2 = vertexPosition(1) - vertexPosition(2);
		Eigen::Vector4f b = hitPoint - vertexPosition(2);

		// ��ȫ��Ԫ����˹��Ԫ
		// ����������
		auto matrixCol1 = _mm_load_ps(a1.data());
		auto matrixCol2 = _mm_load_ps(a2.data());
		auto matrixCol3 = _mm_load_ps(b.data());
		// ����ת�ã���������������
		auto temp1 = _mm_unpacklo_ps(matrixCol1, matrixCol2);
		auto temp2 = _mm_unpackhi_ps(matrixCol1, matrixCol2);
		auto temp3 = _mm_unpacklo_ps(matrixCol3, matrixCol3);
		auto temp4 = _mm_unpackhi_ps(matrixCol3, matrixCol3);
		std::array<__m128, 3> matrixRow;
		matrixRow[0] = _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(temp1), _mm_castps_pd(temp3)));
		matrixRow[1] = _mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(temp1), _mm_castps_pd(temp3)));
		matrixRow[2] = _mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(temp2), _mm_castps_pd(temp4)));
		// ѡȡ��һ����Ԫ����Ԫ�о���ֵ����һ��
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
		// ��һ����Ԫ��һ��
		auto divisor = _mm_broadcast_ss(matrixRow[0].m128_f32);
		matrixRow[0] = _mm_div_ps(matrixRow[0], divisor);
		// ��ȥ�����еĵ�һ��
		for (int i = 1; i < 3; ++i) {
			auto multiplier = _mm_broadcast_ss(matrixRow[i].m128_f32);
			matrixRow[i] = _mm_fnmadd_ps(matrixRow[0], multiplier, matrixRow[i]);
		}
		// ѡȡ�ڶ�����Ԫ
		index = 1;
		if (std::fabsf(matrixRow[2].m128_f32[1]) > std::fabsf(matrixRow[1].m128_f32[1])) {
			index = 2;
		}
		std::swap(matrixRow[1], matrixRow[index]);
		// �ڶ�����Ԫ��һ��
		divisor = _mm_broadcast_ss(&matrixRow[1].m128_f32[1]);
		matrixRow[1] = _mm_div_ps(matrixRow[1], divisor);
		// ���ùܵ����У���ȥ��һ�еĵڶ���
		auto multiplier = _mm_broadcast_ss(&matrixRow[0].m128_f32[1]);
		matrixRow[0] = _mm_fnmadd_ps(matrixRow[1], multiplier, matrixRow[0]);

		// �õ����
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

	// ������ָ���������ķ���
	Eigen::Vector4f tempNormal = (r.direction.dot(normal)) < 0.0f ? normal : -normal;
	std::vector<Eigen::Vector4f> result(diffuseRayNum);

	for (int i = 0; i < diffuseRayNum; ++i) {
		// ��λ���ڵľ��ȷֲ�

		// ˮƽ���������, [0, 2 * PI)
		float phiUnit = rand() & randMask;
		Eigen::Vector4f horizontal(cosTable[phiUnit], sinTable[phiUnit], 0.0f, 0.0f);

		// ��ֱ�����������, [0, PI)
		float thetaUnit = (rand() & randMask) >> 1;

		// �����������
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
		// ��λ���ڵľ��ȷֲ�

		// ˮƽ���������, [0, 2 * PI)
		float phiUnit = rand() & randMask;
		Eigen::Vector4f horizontal(cosTable[phiUnit], sinTable[phiUnit], 0.0f, 0.0f);

		// ��ֱ�����������, [0, PI)
		float thetaUnit = (rand() & randMask) >> 1;

		// �����������
		Eigen::Vector4f offset = horizontal * sinTable[thetaUnit] + cosTable[thetaUnit] * Eigen::Vector4f::UnitZ();

		result[i] = direction + offset * specularRoughness;
	}

	return result;
}

// refer to: https://graphicscompendium.com/raytracing/10-reflection-refraction
std::pair<float, Eigen::Vector4f> Triangle::refract(const Eigen::Vector4f& normal, const Ray& r) const {
	float dot = r.direction.dot(normal);

	// ����������:����������
	float indexRatio = dot > 0.0f ? refractiveIndex : (1.0f / refractiveIndex);

	// ������ָ���������ķ���
	Eigen::Vector4f tempNormal = dot > 0.0f ? -normal : normal;

	// ������ߺͷ�����֮��ļнǵ�����
	float cosineTheta = fabsf(dot);

	// �Ƿ���ȫ����
	float squareSine1 = 1.0f - cosineTheta * cosineTheta;
	float squareSine2 = (indexRatio * indexRatio) * squareSine1;
	if (squareSine2 > 1.0f)
		return std::make_pair(0.0f, Eigen::Vector4f());

	// �������
	float cosine2 = sqrtf(1.0f - squareSine2);
	Eigen::Vector4f outRay = indexRatio * (r.direction + cosineTheta * tempNormal) - tempNormal * cosine2;

	// ��������ı�����Schlick's approximation
	float temp = (1.0f - indexRatio) / (1.0f + indexRatio);
	float r0 = temp * temp;
	float squareTemp = (1.0f - cosineTheta) * (1.0f - cosineTheta);
	float pow5 = squareTemp * squareTemp * (1.0f - cosineTheta);
	float reflectProportion = r0 + (1.0f - r0) * pow5;

	return std::make_pair(1.0f - reflectProportion, outRay);
}
