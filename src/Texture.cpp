#include <RayTracer/Texture.h>
#include <stb_image.h>

Texture::Texture(std::string_view path) {
	int height, channels;
	stbi_set_flip_vertically_on_load(1);
	data = stbi_load(path.data(), &width, &height, &channels, 3);
	if (data == nullptr)
		return;

	uMaxIndex = static_cast<float>(width - 1);
	vMaxIndex = static_cast<float>(height - 1);
}

Texture::~Texture() {
	if (data != nullptr)
		stbi_image_free(data);
}

Texture::Texture(Texture&& rhs) noexcept {
	this->data = rhs.data;
	rhs.data = nullptr;
	this->uMaxIndex = rhs.uMaxIndex;
	this->vMaxIndex = rhs.vMaxIndex;
	this->width = rhs.width;
}

Texture& Texture::operator=(Texture&& rhs) noexcept {
	this->data = rhs.data;
	rhs.data = nullptr;
	this->uMaxIndex = rhs.uMaxIndex;
	this->vMaxIndex = rhs.vMaxIndex;
	this->width = rhs.width;
	return *this;
}

bool Texture::hasTexture() const {
	return data != nullptr;
}

Eigen::Vector4f Texture::sampleTexture(const Eigen::Vector2f& uvCoordinate) const {
	float u = uvCoordinate(0);
	float v = uvCoordinate(1);
	// try nearest-neighbor
	int sampleX = lroundf(u * uMaxIndex);
	int sampleY = lroundf(v * vMaxIndex);
	uint8_t* color = data + (sampleY * width + sampleX) * 3;
	// 映射到[0, 1]，再转Gamma为Linear
	float r = std::powf(color[0] / 255.0f, 2.2f);
	float g = std::powf(color[1] / 255.0f, 2.2f);
	float b = std::powf(color[2] / 255.0f, 2.2f);
	return Eigen::Vector4f(r, g, b, 0.0f);
}