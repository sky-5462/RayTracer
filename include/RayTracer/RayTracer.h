#pragma once

#include <RayTracer/Triangle.h>
#include <RayTracer/Camera.h>
#include <RayTracer/BVH.h>
#include <RayTracer/Texture.h>
#include <Eigen/Core>
#include <string>

class RayTracer {
public:
	RayTracer(int width, int height);
	void setCamera(float cameraX, float cameraY, float cameraZ,
				   float viewPointX, float viewPointY, float viewPointZ,
				   float focal, float rotateAngle);
	// [0, 1]
	void setBackgroundColor(float r, float g, float b);
	void setDiffuseRayNum(int num);
	void setSpecularRayNum(int num);
	void setMaxRecursionDepth(int depth);

	void loadModel(const std::string& path, int index, float offsetX, float offsetY, float offsetZ);
	void loadTexture(const std::string& path, int index);
	void overrideColor(int index, float r, float g, float b);
	void overrideIsMetal(int index, bool isMetal);
	void overrideIsLightEmitting(int index, bool isLightEmitting);
	void overrideIsTransparent(int index, bool isTransparent);
	void overrideSpecularRoughness(int index, float roughness);
	void overrideRefractiveIndex(int index, float refractiveIndex);

	void addTriangle(float x0, float y0, float z0,
					 float x1, float y1, float z1,
					 float x2, float y2, float z2,
					 float nx, float ny, float nz,
					 float r, float g, float b,
					 bool isMetal, bool isLightEmitting, bool isTransparent,
					 float specularRoughness, float refractiveIndex);

	// finish all settings before rendering
	void render(int frames);

private:
	const int width;
	const int height;
	Eigen::Array<Eigen::Vector3f, -1, -1> accumulateImg;
	std::vector<Triangle> trianglesArray;
	Camera camera;
	BVH bvh;

	// default: 10
	int diffuseRayNum;
	// default: 5
	int specualrRayNum;
	// default: 4
	int maxRecursionDepth;
	// default: black
	Eigen::Vector3f backgroundColor;

	std::vector<Texture> texture;

	void renderOneFrame();
	Eigen::Vector3f color(int depth, const Ray& r) const;
};