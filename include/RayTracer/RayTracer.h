#pragma once

#include <RayTracer/Triangle.h>
#include <RayTracer/Camera.h>
#include <RayTracer/BVH.h>
#include<Eigen/Core>

#include <string>

class RayTracer {
public:
	RayTracer(int width, int height);
	void loadModel(const std::string& path);
	int getRenderedNum() const;
	void render(int frames);
	void outputImg() const;

private:
	const int width;
	const int height;
	int renderedNum;
	Eigen::Array<Eigen::Vector3f, -1, -1> accumulateImg;
	Eigen::Array<Eigen::Vector3f, -1, -1> tempImg;    // shared by threads
	std::vector<Triangle> trianglesArray;
	Camera camera;
	BVH bvh;

	void renderOneFrame();
	Eigen::Vector3f color(int depth, const Ray& r) const;
};