#include <RayTracer/RayTracer.h>

#include <array>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cfloat>
#include <chrono>
#include <exception>

#include <assimp/Importer.hpp>
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <Eigen/Geometry>
#include <tbb/tbb.h>
#include <stb_image_write.h>

void RayTracer::loadModel(std::string_view modelPath,
						  const Eigen::Vector4f& origin,
						  bool isMetal,
						  bool isLightEmitting,
						  bool isTransparent,
						  float specularRoughness,
						  float refIndex,
						  const std::optional<std::string_view>& texturePath,
						  const std::optional<Eigen::Vector4f>& color) {
	Assimp::Importer importer;
	importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);
	auto scene = importer.ReadFile(std::string(modelPath),
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
								   aiProcess_GenUVCoords);
	if (scene == nullptr) {
		std::string err("Can't load model in ");
		err += modelPath;
		throw std::exception(err.c_str());
	}

	auto mesh = scene->mMeshes[0];
	unsigned materialIndex = mesh->mMaterialIndex;
	auto material = scene->mMaterials[materialIndex];

	// 确定使用的颜色
	aiColor3D colorTemp;
	material->Get(AI_MATKEY_COLOR_DIFFUSE, colorTemp);
	auto finalColor = color.has_value() ? color.value() : Eigen::Vector4f(colorTemp.r, colorTemp.g, colorTemp.b, 0.0f);

	// 确定是否存在纹理
	bool useTexture = mesh->HasTextureCoords(0) && texturePath.has_value();
	if (useTexture) {
		Texture tex(texturePath.value());
		if (!tex.hasTexture()) {
			useTexture = false;
			std::cout << "Can't load texture in " << texturePath.value() << std::endl;
		}
		else
			texturesArray.push_back(std::move(tex));
	}
	if (!useTexture)
		std::cout << "No texture for model in " << modelPath << std::endl;

	unsigned faceNum = mesh->mNumFaces;
	trianglesArray.reserve(faceNum + trianglesArray.size());
	for (unsigned j = 0; j < faceNum; ++j) {
		const auto& face = mesh->mFaces[j];
		Triangle tri;
		for (int k = 0; k < 3; ++k) {
			unsigned index = face.mIndices[k];

			const auto& vertex = mesh->mVertices[index];
			tri.vertexPosition(k) = Eigen::Vector4f(vertex.x, vertex.y, vertex.z, 0.0f) + origin;

			const auto& normal = mesh->mNormals[index];
			tri.vertexNormal(k) = Eigen::Vector4f(normal.x, normal.y, normal.z, 0.0f);

			if (useTexture) {
				const auto& uvCoordinate = mesh->mTextureCoords[0][index];
				tri.uvCoordinate(k) = Eigen::Vector2f(uvCoordinate.x, uvCoordinate.y);
			}
		}

		tri.planeNormal = (tri.vertexPosition(1) - tri.vertexPosition(0)).cross3(tri.vertexPosition(2) - tri.vertexPosition(0)).normalized();
		tri.isMetal = isMetal;
		tri.isLightEmitting = isLightEmitting;
		tri.isTransparent = isTransparent;
		tri.specularRoughness = specularRoughness;
		tri.refractiveIndex = refIndex;
		tri.color = finalColor;
		tri.textureIndex = useTexture ? texturesArray.size() - 1 : -1;
		trianglesArray.push_back(tri);
	}
}

void RayTracer::addTriangle(const Eigen::Vector4f& vertex0,
							const Eigen::Vector4f& vertex1,
							const Eigen::Vector4f& vertex2,
							const Eigen::Vector4f& normalSide,
							const Eigen::Vector4f& color,
							bool isMetal, bool isLightEmitting, bool isTransparent,
							float specularRoughness, float refractiveIndex) {
	Triangle tri;
	tri.vertexPosition[0] = vertex0;
	tri.vertexPosition[1] = vertex1;
	tri.vertexPosition[2] = vertex2;
	tri.planeNormal = (tri.vertexPosition(1) - tri.vertexPosition(0)).cross3(tri.vertexPosition(2) - tri.vertexPosition(0)).normalized();
	if (normalSide.dot(tri.planeNormal) < 0.0f)
		tri.planeNormal = -tri.planeNormal;

	for (int i = 0; i < 3; ++i) {
		tri.vertexNormal(i) = tri.planeNormal;
	}
	tri.color = color;
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
	if (index == -1) {
		if (skybox.hasSkybox())
			return skybox.sampleBackground(r);
		else
			return backgroundColor;
	}

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

			if (tri.textureIndex < 0)
				return outColor;
			else {
				Eigen::Vector2f uvCoordinate = alpha * tri.uvCoordinate(0) + beta * tri.uvCoordinate(1) +
					(1.0f - (alpha + beta)) * tri.uvCoordinate(2);

				return outColor.cwiseProduct(texturesArray[tri.textureIndex].sampleTexture(uvCoordinate));
			}
		}
	}
}

void RayTracer::render() {
	accumulateImg.resize(height, width);
	accumulateImg.fill(Eigen::Vector4f::Zero());
	outputBuffer.resize(width * height * 3);
	bvh.buildTree(trianglesArray);

	for (int i = 1; i <= renderNum; ++i) {
		auto time1 = std::chrono::system_clock::now();
		// 用RowMajor方式存储，按照行切块
		tbb::parallel_for(0, height,
						  [this, i](size_t row) {
							  for (int col = 0; col < width; ++col) {
								  const auto& ray = camera.getRay(col, row);

								  Eigen::Vector4f temp = Eigen::Vector4f::Zero();
								  for (const auto& r : ray) {
									  temp += color(0, r);
								  }
								  accumulateImg(row, col) += temp * 0.25f;

								  for (int k = 0; k < 3; ++k) {
									  float averaged = accumulateImg(row, col)(k) / i;
									  float gammaCorrected = powf(averaged, 1.0f / 2.2f);
									  int clipNum = lroundf(gammaCorrected * 255.0f);
									  if (clipNum > 255)
										  clipNum = 255;
									  if (clipNum < 0)
										  clipNum = 0;

									  outputBuffer[(row * width + col) * 3 + k] = clipNum;
								  }
							  }
						  });
		auto time2 = std::chrono::system_clock::now();

		std::stringstream str;
		str << "out_";
		str.fill('0');
		str.width(3);
		str << i;
		str << ".png";
		stbi_write_png(str.str().c_str(), width, height, 3, outputBuffer.data(), 3 * width);

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

void RayTracer::parseConfigFile(std::string_view path) {
	std::ifstream config(path.data());
	std::string key;
	if (config >> key && key == "frame")
		config >> width >> height;
	else
		throw std::exception("Can't parse \"frame\"");

	if (config >> key && key == "camera") {
		float x, y, z, atX, atY, atZ, focal, rotate;
		config >> x >> y >> z >> atX >> atY >> atZ >> focal >> rotate;
		camera.setCamera(Eigen::Vector4f(x, y, z, 0.0f),
						 Eigen::Vector4f(atX, atY, atZ, 0.0f),
						 focal, rotate, width, height);
	}
	else
		throw std::exception("Can't parse \"camera\"");

	if (config >> key && key == "background_color") {
		float r, g, b;
		config >> r >> g >> b;
		backgroundColor = Eigen::Vector4f(r, g, b, 0.0f);
	}
	else
		throw std::exception("Can't parse \"background_color\"");

	if (config >> key && key == "max_recursion_depth") {
		config >> maxRecursionDepth;
		if (maxRecursionDepth <= 0)
			throw std::exception("Expect: \"max_recursion_depth\" > 0");
	}
	else
		throw std::exception("Can't parse \"max_recursion_depth\"");

	if (config >> key && key == "diffuse_ray_number") {
		config >> diffuseRayNum;
		if (diffuseRayNum <= 0)
			throw std::exception("Expect: \"diffuse_ray_number\" > 0");
	}
	else
		throw std::exception("Can't parse \"diffuse_ray_number\"");

	if (config >> key && key == "specular_ray_number") {
		config >> specualrRayNum;
		if (specualrRayNum <= 0)
			throw std::exception("Expect: \"specular_ray_number\" > 0");
	}
	else
		throw std::exception("Can't parse \"specular_ray_number\"");


	while (config >> key) {
		if (key == "skybox") {
			float brightness;
			std::string front, back, left, right, top, bottom;
			config >> brightness >> front >> back >> left >> right >> top >> bottom;
			skybox.load(brightness, front, back, left, right, top, bottom);
			if (!skybox.hasSkybox())
				std::cout << "Can't load skybox\n";
		}
		else if (key == "model_start") {
			std::string modelPath;
			if (config >> key && key == "model_path")
				config >> modelPath;
			else
				throw std::exception("Can't parse \"model_path\"");

			std::string texturePath;
			if (config >> key && key == "texture_path")
				config >> texturePath;
			else
				throw std::exception("Can't parse \"texture_path\"");
			auto texPathOptional = std::make_optional(static_cast<std::string_view>(texturePath));
			if (texturePath == "no")
				texPathOptional.reset();

			Eigen::Vector4f origin;
			if (config >> key && key == "position_offset") {
				float x, y, z;
				config >> x >> y >> z;
				origin = Eigen::Vector4f(x, y, z, 0.0f);
			}
			else
				throw std::exception("Can't parse \"position_offset\"");

			bool isMetal;
			if (config >> key && key == "is_metal")
				config >> isMetal;
			else
				throw std::exception("Can't parse \"is_metal\"");

			bool isLightEmitting;
			if (config >> key && key == "is_light_emitting")
				config >> isLightEmitting;
			else
				throw std::exception("Can't parse \"is_light_emitting\"");

			bool isTransparent;
			if (config >> key && key == "is_transparent")
				config >> isTransparent;
			else
				throw std::exception("Can't parse \"is_transparent\"");

			float specularRoughness;
			if (config >> key && key == "specular_roughness")
				config >> specularRoughness;
			else
				throw std::exception("Can't parse \"specular_roughness\"");

			float refIndex;
			if (config >> key && key == "refractive_index")
				config >> refIndex;
			else
				throw std::exception("Can't parse \"refractive_index\"");

			std::optional<Eigen::Vector4f> color;
			config >> key;
			if (key == "override_color") {
				float r, g, b;
				config >> r >> g >> b;
				color = Eigen::Vector4f(r, g, b, 0.0f);

				config >> key;
			}
			if (key == "model_end") {
				loadModel(modelPath, origin, isMetal, isLightEmitting, isTransparent, specularRoughness, refIndex,
						  texPathOptional, color);
				continue;
			}
			else
				throw std::exception("Can't parse \"model_end\"");
		}
		else if (key == "triangle_start") {
			Eigen::Vector4f vertex0;
			if (config >> key && key == "vertex_0") {
				float x, y, z;
				config >> x >> y >> z;
				vertex0 = Eigen::Vector4f(x, y, z, 0.0f);
			}
			else
				throw std::exception("Can't parse \"vertex_0\"");

			Eigen::Vector4f vertex1;
			if (config >> key && key == "vertex_1") {
				float x, y, z;
				config >> x >> y >> z;
				vertex1 = Eigen::Vector4f(x, y, z, 0.0f);
			}
			else
				throw std::exception("Can't parse \"vertex_1\"");

			Eigen::Vector4f vertex2;
			if (config >> key && key == "vertex_2") {
				float x, y, z;
				config >> x >> y >> z;
				vertex2 = Eigen::Vector4f(x, y, z, 0.0f);
			}
			else
				throw std::exception("Can't parse \"vertex_2\"");

			Eigen::Vector4f normalSide;
			if (config >> key && key == "normal_side") {
				float x, y, z;
				config >> x >> y >> z;
				normalSide = Eigen::Vector4f(x, y, z, 0.0f);
			}
			else
				throw std::exception("Can't parse \"normal_side\"");

			Eigen::Vector4f color;
			if (config >> key && key == "color") {
				float r, g, b;
				config >> r >> g >> b;
				color = Eigen::Vector4f(r, g, b, 0.0f);
			}
			else
				throw std::exception("Can't parse \"color\"");

			bool isMetal;
			if (config >> key && key == "is_metal")
				config >> isMetal;
			else
				throw std::exception("Can't parse \"is_metal\"");

			bool isLightEmitting;
			if (config >> key && key == "is_light_emitting")
				config >> isLightEmitting;
			else
				throw std::exception("Can't parse \"is_light_emitting\"");

			bool isTransparent;
			if (config >> key && key == "is_transparent")
				config >> isTransparent;
			else
				throw std::exception("Can't parse \"is_transparent\"");

			float specularRoughness;
			if (config >> key && key == "specular_roughness")
				config >> specularRoughness;
			else
				throw std::exception("Can't parse \"specular_roughness\"");

			float refIndex;
			if (config >> key && key == "refractive_index")
				config >> refIndex;
			else
				throw std::exception("Can't parse \"refractive_index\"");

			if (config >> key && key == "triangle_end") {
				addTriangle(vertex0, vertex1, vertex2, normalSide, color,
							isMetal, isLightEmitting, isTransparent, specularRoughness, refIndex);
				continue;
			}
			else
				throw std::exception("Can't parse \"triangle_end\"");
		}
		else if (key == "render_num") {
			config >> renderNum;
			break;
		}
		else
			throw std::exception("Expect: \"skybox\" or \"model_start\" or \"triangle_start\" or \"render_num\"");
	}

	config.close();
	render();
}