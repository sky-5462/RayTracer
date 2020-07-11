#include <RayTracer/Texture.h>
#include <stb_image.h>

Texture::Texture() : data(nullptr), width(0), uMaxIndex(0.0f), vMaxIndex(0.0f) {}

Texture::~Texture() {
	if (data != nullptr)
		stbi_image_free(data);
}

bool Texture::loadTexture(const std::string& path) {
	int height, channels;
	data = stbi_load(path.c_str(), &width, &height, &channels, 3);
	if (data == nullptr)
		return false;

	uMaxIndex = static_cast<float>(width - 1);
	vMaxIndex = static_cast<float>(height - 1);
	return true;
}

bool Texture::hasTexture() const {
	return data != nullptr;
}

Eigen::Vector4f Texture::sampleTexture(const Eigen::Vector2f& uvCoordinate) const {
	float u = uvCoordinate(0);
	float v = uvCoordinate(1);
	// try nearest-neighbor
	int sampleX = static_cast<int>(roundf(u * uMaxIndex));
	int sampleY = static_cast<int>(roundf(v * vMaxIndex));
	uint8_t* color = data + (sampleY * width + sampleX) * 3;
	float r = static_cast<float>(color[0]) / 255.0f;
	float g = static_cast<float>(color[1]) / 255.0f;
	float b = static_cast<float>(color[2]) / 255.0f;
	return Eigen::Vector4f(r, g, b, 0.0f);
}