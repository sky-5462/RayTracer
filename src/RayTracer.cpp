#include <RayTracer/RayTracer.h>

#include <array>
#include <sstream>
#include <iostream>
#include <cfloat>
#include <chrono>

#include <assimp/Importer.hpp>
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <Eigen/Geometry>
#include <tbb/tbb.h>
#include <stb_image_write.h>

RayTracer::RayTracer(int width, int height) :
	width(width),
	height(height),
	accumulateImg(height, width),
	camera(width, height),
	diffuseRayNum(10),
	specualrRayNum(5),
	maxRecursionDepth(4),
	backgroundColor(Eigen::Vector4f::Zero()),
	texture() {
}

void RayTracer::setBackgroundColor(float r, float g, float b) {
	backgroundColor = Eigen::Vector4f(r, g, b, 0.0f);
}

void RayTracer::setDiffuseRayNum(int num) {
	if (num >= 1)
		diffuseRayNum = num;
}

void RayTracer::setSpecularRayNum(int num) {
	if (num >= 1)
		specualrRayNum = num;
}

void RayTracer::setMaxRecursionDepth(int depth) {
	if (depth >= 1)
		maxRecursionDepth = depth;
}

void RayTracer::loadModel(std::string_view path, int index, float offsetX, float offsetY, float offsetZ) {
	Assimp::Importer importer;
	importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);
	auto scene = importer.ReadFile(std::string(path),
								   aiProcess_GenNormals |
								   aiProcess_Triangulate |
								   aiProcess_FixInfacingNormals |
								   aiProcess_PreTransformVertices |
								   aiProcess_RemoveRedundantMaterials |
								   aiProcess_FindDegenerates |
								   aiProcess_SortByPType |
								   aiProcess_FindInvalidData |
								   aiProcess_OptimizeGraph |
								   aiProcess_OptimizeMeshes |
								   aiProcess_GenUVCoords |
								   aiProcess_FlipUVs);

	if (scene == nullptr) {
		std::cout << "Can't load model\n";
		return;
	}

	auto mesh = scene->mMeshes[0];
	unsigned materialIndex = mesh->mMaterialIndex;
	auto material = scene->mMaterials[materialIndex];

	aiColor3D color;
	material->Get(AI_MATKEY_COLOR_DIFFUSE, color);

	bool hasTextureCoord = mesh->HasTextureCoords(0);
	unsigned faceNum = mesh->mNumFaces;
	trianglesArray.reserve(faceNum + trianglesArray.size());
	for (unsigned j = 0; j < faceNum; ++j) {
		const auto& face = mesh->mFaces[j];
		Triangle tri;
		for (int k = 0; k < 3; ++k) {
			unsigned index = face.mIndices[k];

			const auto& vertex = mesh->mVertices[index];
			tri.vertexPosition(k) = Eigen::Vector4f(vertex.x + offsetX, vertex.y + offsetY, vertex.z + offsetZ, 0.0f);

			const auto& normal = mesh->mNormals[index];
			tri.vertexNormal(k) = Eigen::Vector4f(normal.x, normal.y, normal.z, 0.0f);

			tri.isTextureSupported = hasTextureCoord;
			if (hasTextureCoord) {
				const auto& uvCoordinate = mesh->mTextureCoords[0][index];
				tri.uvCoordinate(k) = Eigen::Vector2f(uvCoordinate.x, uvCoordinate.y);
			}
		}
		tri.planeNormal = (tri.vertexPosition(1) - tri.vertexPosition(0)).cross3(tri.vertexPosition(2) - tri.vertexPosition(0)).normalized();
		tri.color = Eigen::Vector4f(color.r, color.g, color.b, 0.0f);
		tri.isLightEmitting = false;
		tri.isTransparent = false;
		tri.isMetal = false;
		tri.specularRoughness = 1.0f;
		tri.refractiveIndex = 1.5f;
		tri.textureIndex = index;
		trianglesArray.push_back(tri);
	}

	texture.push_back(Texture());
}

void RayTracer::loadTexture(std::string_view path, int index) {
	auto it = std::find_if(trianglesArray.cbegin(), trianglesArray.cend(),
						   [index](const auto& val) {
							   return val.textureIndex == index;
						   });
	if (it->isTextureSupported) {
		bool success = texture[index].loadTexture(path);
		if (!success)
			std::cout << "Failed to load texture #" << index << '\n';
	}
	else {
		std::cout << "Model " << index << " don't have uvCoordinate, no texture\n";
	}
}

void RayTracer::overrideColor(int index, float r, float g, float b) {
	for (auto& tri : trianglesArray) {
		if (tri.textureIndex == index) {
			tri.color = Eigen::Vector4f(r, g, b, 0.0f);
		}
	}
}

void RayTracer::overrideIsMetal(int index, bool isMetal) {
	for (auto& tri : trianglesArray) {
		if (tri.textureIndex == index) {
			tri.isMetal = isMetal;
		}
	}
}

void RayTracer::overrideIsLightEmitting(int index, bool isLightEmitting) {
	for (auto& tri : trianglesArray) {
		if (tri.textureIndex == index) {
			tri.isLightEmitting = isLightEmitting;
		}
	}
}

void RayTracer::overrideIsTransparent(int index, bool isTransparent) {
	for (auto& tri : trianglesArray) {
		if (tri.textureIndex == index) {
			tri.isTransparent = isTransparent;
		}
	}
}

void RayTracer::overrideSpecularRoughness(int index, float roughness) {
	for (auto& tri : trianglesArray) {
		if (tri.textureIndex == index) {
			tri.specularRoughness = roughness;
		}
	}
}

void RayTracer::overrideRefractiveIndex(int index, float refractiveIndex) {
	for (auto& tri : trianglesArray) {
		if (tri.textureIndex == index) {
			tri.refractiveIndex = refractiveIndex;
		}
	}
}

void RayTracer::addTriangle(float x0, float y0, float z0,
							float x1, float y1, float z1,
							float x2, float y2, float z2,
							float nx, float ny, float nz,
							float r, float g, float b,
							bool isMetal, bool isLightEmitting, bool isTransparent,
							float specularRoughness, float refractiveIndex) {
	Triangle tri;
	tri.vertexPosition[0] = Eigen::Vector4f(x0, y0, z0, 0.0f);
	tri.vertexPosition[1] = Eigen::Vector4f(x1, y1, z1, 0.0f);
	tri.vertexPosition[2] = Eigen::Vector4f(x2, y2, z2, 0.0f);
	tri.planeNormal = (tri.vertexPosition(1) - tri.vertexPosition(0)).cross3(tri.vertexPosition(2) - tri.vertexPosition(0)).normalized();
	Eigen::Vector4f normalDirection = Eigen::Vector4f(nx, ny, nz, 0.0f);
	if (normalDirection.dot(tri.planeNormal) < 0.0f)
		tri.planeNormal = -tri.planeNormal;

	for (int i = 0; i < 3; ++i) {
		tri.vertexNormal(i) = tri.planeNormal;
	}
	tri.color = Eigen::Vector4f(r, g, b, 0.0f);
	tri.isMetal = isMetal;
	tri.isLightEmitting = isLightEmitting;
	tri.isTransparent = isTransparent;
	tri.specularRoughness = specularRoughness;
	tri.refractiveIndex = refractiveIndex;
	tri.textureIndex = -1;
	trianglesArray.push_back(tri);
}

Eigen::Vector4f RayTracer::color(int depth, const Ray& r) const {
	const auto& hitList = bvh.hit(r);

	float t = FLT_MAX;
	int index = -1;
	float alpha, beta;
	for (int i = 0; i < hitList.size(); ++i) {
		const auto& tri = trianglesArray[hitList[i]];
		const auto& hitCheck = tri.hit(r);
		if (hitCheck(2) < t) {
			index = hitList[i];
			t = hitCheck(2);
			alpha = hitCheck(0);
			beta = hitCheck(1);
		}
	}

	// no hit
	if (index == -1)
		return backgroundColor;

	const auto& tri = trianglesArray[index];

	Eigen::Vector4f hitPoint = r.origin + t * r.direction;
	Eigen::Vector4f normal = alpha * tri.vertexNormal(0) + beta * tri.vertexNormal(1) +
		(1.0f - (alpha + beta)) * tri.vertexNormal(2);
	normal.normalize();

	// ignore rays coming from the back side
	float cosine = normal.dot(r.direction);
	if (depth == maxRecursionDepth || (!tri.isTransparent && cosine >= 0.0f))
		return Eigen::Vector4f::Zero();

	if (tri.isLightEmitting)
		return tri.color;

	// refer to: https://zhuanlan.zhihu.com/p/21961722?refer=highwaytographics
	if (tri.isMetal) {
		// specular reflection only
		const auto& specularOutRay = tri.specular(normal, r, specualrRayNum);
		Eigen::Vector4f specularColor = Eigen::Vector4f::Zero();
		for (int i = 0; i < specualrRayNum; ++i) {
			specularColor += color(depth + 1, Ray(hitPoint, specularOutRay[i]));
		}
		return specularColor.cwiseProduct(tri.color) / static_cast<float>(specualrRayNum);
	}
	else {
		if (tri.isTransparent) {
			const auto& specularOutRay = tri.specular(normal, r, specualrRayNum);
			Eigen::Vector4f specularColor = Eigen::Vector4f::Zero();
			for (int i = 0; i < specualrRayNum; ++i) {
				specularColor += color(depth + 1, Ray(hitPoint, specularOutRay[i]));
			}
			specularColor /= static_cast<float>(specualrRayNum);

			const auto& [refractProportion, refractOut] = tri.refract(normal, r);
			Eigen::Vector4f refractColor = color(depth + 1, Ray(hitPoint, refractOut));
			return refractProportion * refractColor + (1.0f - refractProportion) * specularColor;
		}
		else {
			const auto& specularOutRay = tri.specular(normal, r, specualrRayNum);
			Eigen::Vector4f specularColor = Eigen::Vector4f::Zero();
			for (int i = 0; i < specualrRayNum; ++i) {
				specularColor += color(depth + 1, Ray(hitPoint, specularOutRay[i]));
			}
			specularColor *= (0.04f / static_cast<float>(specualrRayNum));

			const auto& diffuseOutRay = tri.diffuse(normal, r, diffuseRayNum);
			Eigen::Vector4f diffuseColor = Eigen::Vector4f::Zero();
			for (int i = 0; i < diffuseRayNum; ++i) {
				diffuseColor += color(depth + 1, Ray(hitPoint, diffuseOutRay[i]));
			}
			diffuseColor = diffuseColor.cwiseProduct(tri.color) / static_cast<float>(diffuseRayNum);

			Eigen::Vector4f outColor = specularColor + diffuseColor * fabsf(normal.dot(r.direction));

			int index = tri.textureIndex;
			if (index < 0)
				return outColor;

			const auto& tex = texture[index];
			if (tex.hasTexture()) {
				Eigen::Vector2f uvCoordinate = alpha * tri.uvCoordinate(0) + beta * tri.uvCoordinate(1) +
					(1.0f - (alpha + beta)) * tri.uvCoordinate(2);

				return outColor.cwiseProduct(tex.sampleTexture(uvCoordinate));
			}
			else
				return outColor;
		}
	}

}

void RayTracer::renderOneFrame() {
	tbb::parallel_for(0, width,
					  [this](size_t col) {
						  for (int row = 0; row < height; ++row) {
							  const auto& ray = camera.getRay(col, row);

							  Eigen::Vector4f temp = Eigen::Vector4f::Zero();
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
			accumulateImg(row, col) = Eigen::Vector4f::Zero();
		}
	}

	std::vector<uint8_t> byteBuffer(width * height * 3);
	bvh.buildTree(trianglesArray);
	for (int i = 1; i <= frames; ++i) {
		auto time1 = std::chrono::system_clock::now();
		renderOneFrame();
		auto time2 = std::chrono::system_clock::now();

		// convert float to uint8_t
		for (int col = 0; col < width; ++col) {
			for (int row = 0; row < height; ++row) {
				for (int k = 0; k < 3; ++k) {
					float averaged = accumulateImg(row, col)(k) / static_cast<float>(i);
					float gammaCorrected = sqrtf(averaged);
					int temp = static_cast<int>(gammaCorrected * 255.0f);
					if (temp > 255)
						temp = 255;
					else if (temp < 0)
						temp = 0;

					byteBuffer[row * (3 * width) + col * 3 + k] = temp;
				}
			}
		}

		std::stringstream str;
		str << "out_";
		str.fill('0');
		str.width(3);
		str << i;
		str << ".png";
		stbi_write_png(str.str().c_str(), width, height, 3, byteBuffer.data(), 3 * width);
		
		std::cout << "Output frame " << i << ", use " << std::chrono::duration<float>(time2 - time1).count() << "s\n";
	}
	std::cout << "Render finished" << std::endl;
}

void RayTracer::setCamera(float cameraX, float cameraY, float cameraZ,
						  float viewPointX, float viewPointY, float viewPointZ,
						  float focal, float rotateAngle) {
	camera.setCamera(Eigen::Vector4f(cameraX, cameraY, cameraZ, 0.0f),
					 Eigen::Vector4f(viewPointX, viewPointY, viewPointZ, 0.0f),
					 focal, rotateAngle, width, height);
}