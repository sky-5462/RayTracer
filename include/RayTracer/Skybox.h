#pragma once

#include <RayTracer/Texture.h>
#include <RayTracer/Ray.h>

#include <string_view>
#include <vector>

class Skybox {
public:
	void load(float brightness,
			  std::string_view frontPath,
			  std::string_view backPath,
			  std::string_view leftPath,
			  std::string_view rightPath,
			  std::string_view topPath,
			  std::string_view bottomPath);

	bool hasSkybox() const;
	Eigen::Vector4f sampleBackground(const Ray& ray) const;

private:
	std::vector<Texture> backgroundImg;
	float brightness;
};
