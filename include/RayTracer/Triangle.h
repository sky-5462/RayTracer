#pragma once

#include <RayTracer/Ray.h>
#include <Eigen/Core>

class Triangle {
public:
	// OpenGL coordinate
	Eigen::Array<Eigen::Vector3f, 3, 1> vertexPosition;
	// calculate hit point normal by interpolation
	Eigen::Array<Eigen::Vector3f, 3, 1> vertexNormal;
	// used for hit checking
	Eigen::Vector3f planeNormal;

	bool isLightEmitting;
	bool isMetal;

	// [0, 1], diffuse color for non-metal or specular color for metal
	Eigen::Vector3f color;

	float specularRoughness;

	// control refraction
	float refractiveIndex;
	bool isTransparent;

	int textureIndex;
	Eigen::Array<Eigen::Vector2f, 3, 1> uvCoordinate;

	// return t, alpha, beta
	std::tuple<float, float, float> hit(const Ray& r) const;

	// generate multiple rays
	std::vector<Eigen::Vector3f> diffuse(const Eigen::Vector3f& normal, const Ray& r, int diffuseRayNum) const;

	// an exact specular reflection ray
	std::vector<Eigen::Vector3f> specular(const Eigen::Vector3f& normal, const Ray& r, int specularRayNum) const;

	// return the proportion of refraction and the corresponding ray
	std::pair<float, Eigen::Vector3f> refract(const Eigen::Vector3f& normal, const Ray& r) const;
};