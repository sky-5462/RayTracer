#pragma once

#include <Eigen/Core>
#include <string_view>

// assume it's RGB
class Texture {
public:
	Texture(std::string_view path);
	~Texture();
	Texture(Texture&& rhs) noexcept;
	Texture& operator=(Texture&& rhs) noexcept;

	bool hasTexture() const;
	Eigen::Vector4f sampleTexture(const Eigen::Vector2f& uvCoordinate) const;

private:
	uint8_t* data;
	float uMaxIndex;
	float vMaxIndex;
	int width;
};