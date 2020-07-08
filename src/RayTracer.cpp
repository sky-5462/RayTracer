#include <RayTracer/RayTracer.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <array>
#include <sstream>
#include <iostream>
#include <memory>

#include <assimp/Importer.hpp>
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <Eigen/Geometry>

#include <tbb/tbb.h>

RayTracer::RayTracer(int width, int height) :
	width(width),
	height(height),
	accumulateImg(height, width),
	camera(width, height),
	diffuseRayNum(10),
	maxRecursionDepth(4),
	backgroundColor(Eigen::Vector3f::Zero()) {
}

void RayTracer::setBackgroundColor(float r, float g, float b) {
	backgroundColor = Eigen::Vector3f(r, g, b);
}

void RayTracer::setDiffuseRayNum(int num) {
	diffuseRayNum = num;
}

void RayTracer::setMaxRecursionDepth(int depth) {
	maxRecursionDepth = depth;
}

void RayTracer::loadModel(const std::string& path) {
	Assimp::Importer importer;
	auto scene = importer.ReadFile(path,
								   aiProcess_GenNormals |
								   aiProcess_Triangulate |
								   aiProcess_FixInfacingNormals |
								   aiProcess_PreTransformVertices |
								   aiProcess_FindDegenerates |
								   aiProcess_SortByPType |
								   aiProcess_OptimizeGraph |
								   aiProcess_OptimizeMeshes);

	if (scene == nullptr)
		return;

	unsigned meshNum = scene->mNumMeshes;
	for (unsigned i = 0; i < meshNum; ++i) {
		auto mesh = scene->mMeshes[i];
		if (mesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE)
			continue;

		unsigned faceNum = mesh->mNumFaces;
		for (unsigned j = 0; j < faceNum; ++j) {
			const auto& face = mesh->mFaces[j];
			Triangle tri;
			for (int k = 0; k < 3; ++k) {
				unsigned index = face.mIndices[k];
				const auto& vertex = mesh->mVertices[index];
				tri.vertexPosition(k) = Eigen::Vector3f(vertex.x, vertex.y, vertex.z);
				const auto& normal = mesh->mNormals[index];
				tri.vertexNormal(k) = Eigen::Vector3f(normal.x, normal.y, normal.z);
			}
			tri.planeNormal = (tri.vertexPosition(1) - tri.vertexPosition(0)).cross(tri.vertexPosition(2) - tri.vertexPosition(0)).normalized();
			tri.color = Eigen::Vector3f(0.8f, 0.8f, 0.8f);
			tri.isLightEmitting = false;
			tri.isTransparent = false;
			tri.roughness = 0.5f;
			tri.refractiveIndex = 1.5f;
			trianglesArray.push_back(tri);
		}
	}
}

void RayTracer::addLighting(float x0, float y0, float z0,
							float x1, float y1, float z1,
							float x2, float y2, float z2,
							float r, float g, float b) {
	Triangle tri;
	tri.vertexPosition[0] = Eigen::Vector3f(x0, y0, z0);
	tri.vertexPosition[1] = Eigen::Vector3f(x1, y1, z1);
	tri.vertexPosition[2] = Eigen::Vector3f(x2, y2, z2);
	tri.planeNormal = (tri.vertexPosition(1) - tri.vertexPosition(0)).cross(tri.vertexPosition(2) - tri.vertexPosition(0)).normalized();
	tri.color = Eigen::Vector3f(r, g, b);
	tri.isLightEmitting = true;
	trianglesArray.push_back(tri);
}

Eigen::Vector3f RayTracer::color(int depth, const Ray& r) const {
	if (depth >= maxRecursionDepth)
		return backgroundColor;

	const auto& hitList = bvh.hit(r);
	if (hitList.empty())
		return backgroundColor;

	float t = FLT_MAX;
	int index = -1;
	Eigen::Vector3f normal;
	for (int i = 0; i < hitList.size(); ++i) {
		const auto& tri = trianglesArray[hitList[i]];
		const auto& [temp, normalTemp] = tri.hit(r);
		if (temp < t) {
			index = hitList[i];
			t = temp;
			normal = normalTemp;
		}
	}

	// no hit
	if (index == -1)
		return backgroundColor;

	const auto& tri = trianglesArray[index];
	if (tri.isLightEmitting)
		return tri.color;
	else {
		Eigen::Vector3f hitPoint = r.origin + t * r.direction;

		Eigen::Vector3f refractColor = Eigen::Vector3f::Zero();
		float refractProportion = 0.0f;
		if (tri.isTransparent) {
			Eigen::Vector3f refractOut;
			std::tie(refractProportion, refractOut) = tri.refract(normal, r);

			if (refractProportion > 0.01f)
				refractColor = color(depth + 1, Ray(hitPoint, refractOut));
			else
				refractProportion = 0.0f;

			if (refractProportion > 0.99f)
				return refractColor;
		}

		Eigen::Vector3f reflectColor;
		if (tri.roughness > 0.99f) {
			const auto& diffuseOutRay = tri.diffuse(normal, r, hitPoint, diffuseRayNum);
			Eigen::Vector3f diffuseColor = Eigen::Vector3f::Zero();
			for (int i = 0; i < diffuseRayNum; ++i) {
				diffuseColor += tri.color.cwiseProduct(color(depth + 1, Ray(hitPoint, diffuseOutRay[i])));
			}
			reflectColor = diffuseColor / static_cast<float>(diffuseRayNum);
		}
		else if (tri.roughness < 0.01f) {
			const auto& specularOutRay = tri.specular(normal, r);
			reflectColor = tri.color.cwiseProduct(color(depth + 1, Ray(hitPoint, specularOutRay)));
		}
		else {
			const auto& diffuseOutRay = tri.diffuse(normal, r, hitPoint, diffuseRayNum);
			Eigen::Vector3f diffuseColor = Eigen::Vector3f::Zero();
			for (int i = 0; i < diffuseRayNum; ++i) {
				diffuseColor += tri.color.cwiseProduct(color(depth + 1, Ray(hitPoint, diffuseOutRay[i])));
			}
			diffuseColor /= static_cast<float>(diffuseRayNum);

			const auto& specularOutRay = tri.specular(normal, r);
			Eigen::Vector3f specularColor = tri.color.cwiseProduct(color(depth + 1, Ray(hitPoint, specularOutRay)));

			reflectColor = tri.roughness * diffuseColor + (1.0f - tri.roughness) * specularColor;
		}

		return refractProportion * refractColor + (1.0f - refractProportion) * reflectColor;
	}
}

void RayTracer::renderOneFrame() {
	tbb::parallel_for(0, width,
					  [this](size_t col) {
						  for (int row = 0; row < height; ++row) {
							  const auto& ray = camera.getRay(col, row);

							  Eigen::Vector3f temp = Eigen::Vector3f::Zero();
							  for (const auto& r : ray) {
								  temp += color(0, r);
							  }
							  accumulateImg(row, col) += temp * 0.25f;
						  }
					  });
}

void RayTracer::render(int frames) {
	for (int col = 0; col < width; ++col) {
		for (int row = 0; row < height; ++row) {
			accumulateImg(row, col) = Eigen::Vector3f::Zero();
		}
	}

	auto byteBuffer = std::make_unique<uint8_t[]>(width * height * 3);
	bvh.buildTree(trianglesArray);
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

		stbi_write_png("out.png", width, height, 3, byteBuffer.get(), 3 * width);
		std::cout << "Output frame " << i << '\n';
	}
	std::cout << "Render finished" << std::endl;
}

void RayTracer::setCamera(float cameraX, float cameraY, float cameraZ,
						  float viewPointX, float viewPointY, float viewPointZ,
						  float focal) {
	camera.setCamera(Eigen::Vector3f(cameraX, cameraY, cameraZ),
					 Eigen::Vector3f(viewPointX, viewPointY, viewPointZ),
					 focal, width, height);
}