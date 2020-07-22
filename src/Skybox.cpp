#include <RayTracer/Skybox.h>
#include <array>

void Skybox::load(float brightness,
				  std::string_view frontPath,
				  std::string_view backPath,
				  std::string_view leftPath,
				  std::string_view rightPath,
				  std::string_view topPath,
				  std::string_view bottomPath) {

	this->brightness = brightness;
	std::array<std::string_view, 6> paths({ frontPath, backPath,
										  leftPath, rightPath,
										  topPath, bottomPath });
	backgroundImg.reserve(6);
	for (int i = 0; i < 6; ++i) {
		Texture tex(paths[i]);
		if (tex.hasTexture())
			backgroundImg.push_back(std::move(tex));
		else {
			backgroundImg.clear();
			return;
		}
	}
}

bool Skybox::hasSkybox() const {
	return !backgroundImg.empty();
}

Eigen::Vector4f Skybox::sampleBackground(const Ray& ray) const {
	Eigen::Vector4f projection;
	float divisor, u, v;
	for (int i = 0; i < 6; ++i) {
		// 选取坐标轴，伸缩方向向量到交于天空盒平面
		switch (i) {
		case 0:  // front, ZPos
			divisor = ray.direction(2);
			projection = ray.direction / divisor;
			u = (1.0f - projection(0)) * 0.5f;
			v = (1.0f + projection(1)) * 0.5f;
			break;
		case 1:  // back, ZNeg
			divisor = -ray.direction(2);
			projection = ray.direction / divisor;
			u = (1.0f + projection(0)) * 0.5f;
			v = (1.0f + projection(1)) * 0.5f;
			break;
		case 2:  // left, XNeg
			divisor = -ray.direction(0);
			projection = ray.direction / divisor;
			u = (1.0f - projection(2)) * 0.5f;
			v = (1.0f + projection(1)) * 0.5f;
			break;
		case 3:  // right, XPos
			divisor = ray.direction(0);
			projection = ray.direction / divisor;
			u = (projection(2) + 1.0f) * 0.5f;
			v = (projection(1) + 1.0f) * 0.5f;
			break;
		case 4:  // top, YPos
			divisor = ray.direction(1);
			projection = ray.direction / divisor;
			u = (projection(0) + 1.0f) * 0.5f;
			v = (projection(2) + 1.0f) * 0.5f;
			break;
		case 5:  // bottom, YNeg
			divisor = -ray.direction(1);
			projection = ray.direction / divisor;
			u = (projection(0) + 1.0f) * 0.5f;
			v = (projection(2) - 1.0f) * 0.5f;
			break;
		}

		// 测试是否相交
		if (divisor < 0.0f || u < 0.0f || v < 0.0f || u > 1.0f || v > 1.0f)
			continue;

		// 采样
		return backgroundImg[i].sampleTexture(Eigen::Vector2f(u, v)) * brightness;
	}
	return Eigen::Vector4f::Zero();
}