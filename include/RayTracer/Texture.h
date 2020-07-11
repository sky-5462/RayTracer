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
	Eigen::Vector4f sampleTexture(const Eigen::Vector2f& uvCoordinate) const;

private:
	uint8_t* data;
	float uMaxIndex;
	float vMaxIndex;
	int width;
};