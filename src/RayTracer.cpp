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
	tri.vertex[0] = Eigen::Vector3f(1, 0, -2);
	tri.vertex[1] = Eigen::Vector3f(-1, 0, -2);
	tri.vertex[2] = Eigen::Vector3f(0, 1, -2);
	tri.normal = Eigen::Vector3f(0, 0, 1);
	tri.roughness = 0;

	trianglesArray.push_back(tri);
}

void RayTracer::renderOneFrame() {
	// test
	for (int col = 0; col < width; ++col) {
		for (int row = 0; row < height; ++row) {
			auto ray = camera.getRay(col, row);
			auto& triangle = trianglesArray.front();

			if (triangle.hit(ray))
				resultImg(row, col) = Eigen::Vector3f(1, 1, 1);
			else
				resultImg(row, col) = Eigen::Vector3f(0, 0, 0);

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
	stbi_write_png((prefix + str.str() + ".png").c_str(), width, height, 3, byteBuffer, 3 * width);
	delete[] byteBuffer;
}
