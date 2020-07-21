#pragma once

#include <RayTracer/Ray.h>
#include <Eigen/Core>

class Triangle {
public:
	Eigen::Array<Eigen::Vector4f, 3, 1> vertexPosition;

	// ���㷨������ֵ�õ������������ν���ķ�����
	Eigen::Array<Eigen::Vector4f, 3, 1> vertexNormal;

	// ƽ�淨���������������Ƿ����������ཻ
	Eigen::Vector4f planeNormal;

	// [0, 1], ��������ɫ���ǽ��������淴����ɫ��������
	Eigen::Vector4f color;

	// ����ӳ���UV����
	Eigen::Array<Eigen::Vector2f, 3, 1> uvCoordinate;

	// ���淴��Ĺ⻬��
	float specularRoughness;

	// ������
	float refractiveIndex;

	// ָ��ʹ�õ�����
	int textureIndex;
	bool isTextureSupported;

	bool isLightEmitting;
	bool isMetal;
	bool isTransparent;

	// return [alpha, beta, t]
	Eigen::Vector4f hit(const Ray& r) const;

	// ���ض����������
	std::vector<Eigen::Vector4f> diffuse(const Eigen::Vector4f& normal, const Ray& r, int diffuseRayNum) const;
	std::vector<Eigen::Vector4f> specular(const Eigen::Vector4f& normal, const Ray& r, int specularRayNum) const;

	// ������������Լ�����ռ�ı���
	std::pair<float, Eigen::Vector4f> refract(const Eigen::Vector4f& normal, const Ray& r) const;
};