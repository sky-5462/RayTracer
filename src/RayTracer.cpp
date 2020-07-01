#include <RayTracer/RayTracer.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <array>
#include <sstream>
#include <iostream>

#define MAX_RECURSIVE_DEPTH 10

RayTracer::RayTracer(int width, int height) :
	width(width),
	height(height),
	accumulateImg(height, width),
	tempImg(height, width),
	camera(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), 90, width, height) {
}

void RayTracer::loadModel(const std::string& path) {
	// test
	Triangle tri;
	tri.vertex[0] = Eigen::Vector3f(5, -5, -10);
	tri.vertex[1] = Eigen::Vector3f(-5, -5, -10);
	tri.vertex[2] = Eigen::Vector3f(0, 5, -10);
	tri.normal = Eigen::Vector3f(0, 0, 1);
	tri.color = Eigen::Vector3f(0.9f, 0.1f, 0.3f);
	tri.isLightEmitting = true;
	tri.isTransparent = false;

	trianglesArray.push_back(tri);

	tri.vertex[0] = Eigen::Vector3f(-2, -2, -2);
	tri.vertex[1] = Eigen::Vector3f(2, -2, -2);
	tri.vertex[2] = Eigen::Vector3f(0, -2, -10);
	tri.normal = Eigen::Vector3f(0, 1, 0);
	tri.color = Eigen::Vector3f(0.9f, 0.9f, 0.9f);
	tri.isLightEmitting = false;
	tri.roughness = 0.5f;

	trianglesArray.push_back(tri);

	tri.vertex[0] = Eigen::Vector3f(5, 2, -7);
	tri.vertex[1] = Eigen::Vector3f(-5, 2, -7);
	tri.vertex[2] = Eigen::Vector3f(0, 7, -7);
	tri.normal = Eigen::Vector3f(0, 0, 1);
	tri.color = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
	tri.isLightEmitting = false;
	tri.roughness = 0.0f;
	tri.isTransparent = true;
	tri.refractiveIndex = 1.5f;

	trianglesArray.push_back(tri);
}

Eigen::Vector3f RayTracer::color(int depth, const Ray& r) const {
	if (depth >= MAX_RECURSIVE_DEPTH)
		return Eigen::Vector3f(0.0f, 0.0f, 0.0f);

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
		return Eigen::Vector3f(0.0f, 0.0f, 0.0f);

	auto& tri = trianglesArray[index];
	if (tri.isLightEmitting)
		return tri.color;
	else {
		Eigen::Vector3f hitPoint = r.origin + t * r.direction;

		Eigen::Vector3f refractColor = Eigen::Vector3f::Zero();
		float refractProportion = 0.0f;
		if (tri.isTransparent) {
			Ray refractOut;
			std::tie(refractProportion, refractOut) = tri.refract(r, hitPoint);

			if (refractProportion > 0.01f)
				refractColor = color(depth + 1, refractOut);
			else
				refractProportion = 0.0f;

			if (refractProportion > 0.99f)
				return refractColor;
		}

		Eigen::Vector3f reflectColor;
		if (tri.roughness > 0.99f) {
			auto diffuseOut = tri.diffuse(r, hitPoint);
			Eigen::Vector3f diffuseColor = Eigen::Vector3f::Zero();
			for (int i = 0; i < diffuseOut.size(); ++i) {
				diffuseColor += tri.color.cwiseProduct(color(depth + 1, diffuseOut[i]));
			}
			reflectColor = diffuseColor / static_cast<float>(diffuseOut.size());
		}
		else if (tri.roughness < 0.01f) {
			auto specularOut = tri.specular(r, hitPoint);
			reflectColor = tri.color.cwiseProduct(color(depth + 1, specularOut));
		}
		else {
			auto diffuseOut = tri.diffuse(r, hitPoint);
			Eigen::Vector3f diffuseColor = Eigen::Vector3f::Zero();
			for (int i = 0; i < diffuseOut.size(); ++i) {
				diffuseColor += tri.color.cwiseProduct(color(depth + 1, diffuseOut[i]));
			}
			diffuseColor /= static_cast<float>(diffuseOut.size());

			auto specularOut = tri.specular(r, hitPoint);
			Eigen::Vector3f specularColor = tri.color.cwiseProduct(color(depth + 1, specularOut));

			reflectColor = tri.roughness * diffuseColor + (1 - tri.roughness) * specularColor;
		}

		return refractProportion * refractColor + (1.0f - refractProportion) * reflectColor;
	}
}

void RayTracer::renderOneFrame() {
	for (int col = 0; col < width; ++col) {
		for (int row = 0; row < height; ++row) {
			auto ray = camera.getRay(col, row);

			Eigen::Vector3f temp = Eigen::Vector3f::Zero();
			for (auto& r : ray) {
				temp += color(0, r);
			}
			temp /= 4.0f;

			tempImg(row, col) = temp;
		}
	}

	accumulateImg += tempImg;
}

void RayTracer::render(int frames) {
	for (int col = 0; col < width; ++col) {
		for (int row = 0; row < height; ++row) {
			accumulateImg(row, col) = Eigen::Vector3f::Zero();
		}
	}

	auto byteBuffer = new uint8_t[width * height * 3];
	stbi_flip_vertically_on_write(1);

	for (int i = 1; i <= frames; ++i) {
		renderOneFrame();

		// convert float to uint8_t
		for (int col = 0; col < width; ++col) {
			for (int row = 0; row < height; ++row) {
				for (int k = 0; k < 3; ++k) {
					int temp = static_cast<int>(accumulateImg(row, col)(k) * (255.0f / i));
					if (temp > 255)
						temp = 255;
					else if (temp < 0)
						temp = 0;

					byteBuffer[row * (3 * width) + col * 3 + k] = temp;
				}
			}
		}

		stbi_write_png("out.png", width, height, 3, byteBuffer, 3 * width);
		std::cout << "Output frame " << i << '\n';
	}
	delete[] byteBuffer;
	std::cout << "Render finished" << std::endl;
}
