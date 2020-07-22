#pragma once

#include <RayTracer/Triangle.h>
#include <RayTracer/Camera.h>
#include <RayTracer/BVH.h>
#include <RayTracer/Texture.h>
#include <RayTracer/Skybox.h>
#include <Eigen/Core>
#include <string_view>
#include <optional>

class RayTracer {
public:
	void parseConfigFile(std::string_view path);

private:
	int width;
	int height;
	int renderNum;
	Eigen::Array<Eigen::Vector4f, -1, -1> accumulateImg;
	std::vector<Triangle> trianglesArray;
	std::vector<Texture> texturesArray;
	Camera camera;
	BVH bvh;
	Skybox skybox;

	int diffuseRayNum;
	int specualrRayNum;
	int maxRecursionDepth;
	Eigen::Vector4f backgroundColor;

	void setCamera(float cameraX, float cameraY, float cameraZ,
				   float viewPointX, float viewPointY, float viewPointZ,
				   float focal, float rotateAngle);

	void loadModel(std::string_view modelPath,
				   const Eigen::Vector4f& origin,
				   bool isMetal,
				   bool isLightEmitting,
				   bool isTransparent,
				   float specularRoughness,
				   float refIndex,
				   const std::optional<std::string_view>& texturePath,
				   const std::optional<Eigen::Vector4f>& color);

	void addTriangle(const Eigen::Vector4f& vertex0,
					 const Eigen::Vector4f& vertex1,
					 const Eigen::Vector4f& vertex2,
					 const Eigen::Vector4f& normalSide,
					 const Eigen::Vector4f& color,
					 bool isMetal, bool isLightEmitting, bool isTransparent,
					 float specularRoughness, float refractiveIndex);

	void render();
	Eigen::Vector4f color(int depth, const Ray& r) const;
};