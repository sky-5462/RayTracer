#pragma once

#include <RayTracer/Ray.h>
#include <Eigen/Core>
#include <random>

class Triangle {
public:
	// OpenGL coordinate
	Eigen::Array<Eigen::Vector3f, 3, 1> vertex;
	
	// must be unit vector
	Eigen::Vector3f normal;

	// [0, 1]
	Eigen::Vector3f color;

	// proprotion between diffused and specular reflection
	float roughness;

	// control refraction
	float refractiveIndex;
	bool isTransparent;

	bool isLightEmitting;

	// texture coordinate for each vertex
	int textureIndex;
	Eigen::Array<Eigen::Vector2i, 3, 1> texture;

	static std::mt19937 rand;

	std::pair<bool, float> hit(const Ray& r) const;

	// generate multiple rays
	std::vector<Ray> diffuse(const Ray& r, const Eigen::Vector3f& hitPoint) const;

	// exact specular reflection
	Ray specular(const Ray& r, const Eigen::Vector3f& hitPoint) const;

	// return the proportion of refraction and the corresponding ray
	std::pair<float, Ray> refract(const Ray& r, const Eigen::Vector3f& hitPoint) const;
};