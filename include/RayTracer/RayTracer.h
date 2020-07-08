#pragma once

#include <RayTracer/Triangle.h>
#include <RayTracer/Camera.h>
#include <RayTracer/BVH.h>
#include <Eigen/Core>

#include <string>

class RayTracer {
public:
	RayTracer(int width, int height);
	void setCamera(float cameraX, float cameraY, float cameraZ,
				   float viewPointX, float viewPointY, float viewPointZ,
				   float focal);
	void loadModel(const std::string& path);
	void addLighting(float x0, float y0, float z0,
					 float x1, float y1, float z1,
					 float x2, float y2, float z2,
					 float r, float g, float b);
	// [0, 1]
	void setBackgroundColor(float r, float g, float b);
	void setDiffuseRayNum(int num);
	void setMaxRecursionDepth(int depth);

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
	// default: 4
	int maxRecursionDepth;
	// default: black
	Eigen::Vector3f backgroundColor;

	void renderOneFrame();
	Eigen::Vector3f color(int depth, const Ray& r) const;
};