#include <RayTracer/RayTracer.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <array>
#include <sstream>

RayTracer::RayTracer(int width, int height) :
	width(width),
	height(height),
	resultImg(height, width),
	tempImg(height, width),
	camera(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), 90, width, height),
	renderedNum(0) {
}

void RayTracer::loadModel(const std::string& path) {
	// test
	Triangle tri;
	tri.vertex[0] = Eigen::Vector3f(1, -1, -2);
	tri.vertex[1] = Eigen::Vector3f(-1, -1, -2);
	tri.vertex[2] = Eigen::Vector3f(0, 1, -2);
	tri.normal = Eigen::Vector3f(0, 0, 1);
	tri.roughness = 0;
	tri.color = Eigen::Vector3f(0.9f, 0.1f, 0.3f);
	tri.isLightEmitting = true;

	trianglesArray.push_back(tri);

	tri.vertex[0] = Eigen::Vector3f(-2, -2, -2);
	tri.vertex[1] = Eigen::Vector3f(2, -2, -2);
	tri.vertex[2] = Eigen::Vector3f(0, -2, -10);
	tri.normal = Eigen::Vector3f(0, 1, 0);
	tri.color = Eigen::Vector3f(0.1f, 0.4f, 0.35f);

	trianglesArray.push_back(tri);
}

Eigen::Vector3f RayTracer::color(int depth, const Ray& r) const {
	//auto indics = bvh.hit(r);
	//if (indics.empty())
	//	return Eigen::Vector3f(0, 0, 0);

	//auto t = FLT_MAX;
	//auto index = -1;
	//for (auto i : indics) {
	//	auto& tri = trianglesArray[i];
	//	auto [didHit, temp] = tri.hit(r);
	//	if (didHit && temp < t) {
	//		index = i;
	//		t = temp;
	//	}
	//}

	//if (index == -1)
	//	return Eigen::Vector3f(0, 0, 0);

	//auto& tri = trianglesArray[index];
	//if (tri.isLightEmitting)
	//	return tri.color;

	auto t = FLT_MAX;
	auto index = -1;
	for (int i = 0; i < trianglesArray.size(); ++i) {
		auto& tri = trianglesArray[i];
		auto [didHit, temp] = tri.hit(r);
		if (didHit && temp < t) {
			index = i;
			t = temp;
		}
	}

	if (index == -1)
		return Eigen::Vector3f(0, 0, 0);

	auto& tri = trianglesArray[index];
	if (tri.isLightEmitting)
		return tri.color;

	return Eigen::Vector3f(0, 0, 0);
}

void RayTracer::renderOneFrame() {
	// test
	for (int col = 0; col < width; ++col) {
		for (int row = 0; row < height; ++row) {
			auto ray = camera.getRay(col, row);

			Eigen::Vector3f temp = Eigen::Vector3f::Zero();
			for (auto& r : ray) {
				temp += color(0, r);
			}
			temp /= 4.0f;

			resultImg(row, col) = temp;
		}
	}
}

int RayTracer::getRenderedNum() const {
	return renderedNum;
}

void RayTracer::outputImg() const {
	auto byteBuffer = new uint8_t[width * height * 3];
	// convert float to uint8_t
	for (int col = 0; col < width; ++col) {
		for (int row = 0; row < height; ++row) {
			for (int k = 0; k < 3; ++k) {
				int temp = static_cast<int>(resultImg(row, col)(k) * 255.0f);
				if (temp > 255)
					temp = 255;
				else if (temp < 0)
					temp = 0;

				byteBuffer[row * (3 * width) + col * 3 + k] = temp;
			}
		}
	}

	auto prefix = std::string("out_");
	std::stringstream str;
	str.fill('0');
	str.width(3);
	str << renderedNum;
	stbi_flip_vertically_on_write(1);
	stbi_write_png((prefix + str.str() + ".png").c_str(), width, height, 3, byteBuffer, 3 * width);
	delete[] byteBuffer;
}
