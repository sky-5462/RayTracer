#pragma once

#include <Eigen/Core>
#include <string>

// assume it's RGB
class Texture {
public:
	Texture();
	~Texture();
	bool loadTexture(const std::string& path);
	bool hasTexture() const;
	Eigen::Vector3f sampleTexture(const Eigen::Vector2f& uvCoordinate) const;

private:
	float uMaxIndex;
	float vMaxIndex;
	int width;
	uint8_t* data;
};