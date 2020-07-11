#pragma once

#include <RayTracer/Ray.h>
#include <Eigen/Core>

class Triangle {
public:
	// OpenGL coordinate
	Eigen::Array<Eigen::Vector4f, 3, 1> vertexPosition;
	// calculate hit point normal by interpolation
	Eigen::Array<Eigen::Vector4f, 3, 1> vertexNormal;
	// used for hit checking
	Eigen::Vector4f planeNormal;
	// [0, 1], diffuse color for non-metal or specular color for metal
	Eigen::Vector4f color;

	Eigen::Array<Eigen::Vector2f, 3, 1> uvCoordinate;

	float specularRoughness;
	float refractiveIndex;

	int textureIndex;

	bool isLightEmitting;
	bool isMetal;
	bool isTransparent;

	// return t, alpha, beta
	std::tuple<float, float, float> hit(const Ray& r) const;

	// generate multiple rays
	std::vector<Eigen::Vector4f> diffuse(const Eigen::Vector4f& normal, const Ray& r, int diffuseRayNum) const;

	// an exact specular reflection ray
	std::vector<Eigen::Vector4f> specular(const Eigen::Vector4f& normal, const Ray& r, int specularRayNum) const;

	// return the proportion of refraction and the corresponding ray
	std::pair<float, Eigen::Vector4f> refract(const Eigen::Vector4f& normal, const Ray& r) const;
};