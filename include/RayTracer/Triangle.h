#pragma once

#include <RayTracer/Ray.h>
#include <Eigen/Core>

class Triangle {
public:
	Eigen::Array<Eigen::Vector4f, 3, 1> vertexPosition;

	// 顶点法向量插值得到光线与三角形交点的法向量
	Eigen::Array<Eigen::Vector4f, 3, 1> vertexNormal;

	// 平面法向量用来检查光线是否与三角形相交
	Eigen::Vector4f planeNormal;

	// [0, 1], 漫反射颜色（非金属）或镜面反射颜色（金属）
	Eigen::Vector4f color;

	// 纹理映射的UV坐标
	Eigen::Array<Eigen::Vector2f, 3, 1> uvCoordinate;

	// 镜面反射的光滑度
	float specularRoughness;

	// 折射率
	float refractiveIndex;

	// 指定使用的纹理
	int textureIndex;
	bool isTextureSupported;

	bool isLightEmitting;
	bool isMetal;
	bool isTransparent;

	// return t, alpha, beta
	std::tuple<float, float, float> hit(const Ray& r) const;

	// 返回多条射出光线
	std::vector<Eigen::Vector4f> diffuse(const Eigen::Vector4f& normal, const Ray& r, int diffuseRayNum) const;
	std::vector<Eigen::Vector4f> specular(const Eigen::Vector4f& normal, const Ray& r, int specularRayNum) const;

	// 返回射出光线以及折射占的比例
	std::pair<float, Eigen::Vector4f> refract(const Eigen::Vector4f& normal, const Ray& r) const;
};