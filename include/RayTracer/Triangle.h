#pragma once

#include <RayTracer/Ray.h>
#include <Eigen/Core>

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

	float hit(const Ray& r) const;

	// generate multiple rays
	std::vector<Eigen::Vector3f> diffuse(const Ray& r, const Eigen::Vector3f& hitPoint, int rayNum) const;

	// an exact specular reflection ray
	Eigen::Vector3f specular(const Ray& r) const;

	// return the proportion of refraction and the corresponding ray
	std::pair<float, Eigen::Vector3f> refract(const Ray& r) const;
};