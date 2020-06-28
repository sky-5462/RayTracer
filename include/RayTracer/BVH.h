#pragma once

#include <RayTracer/Triangle.h>
#include <RayTracer/Ray.h>
#include <vector>

struct AABB {
	Eigen::Vector3f min;
	Eigen::Vector3f max;
};

struct Node {
	AABB aabb;
	int vertexIndex;
	Node* left;
	Node* right;

	Node();
	~Node();
};

class BVH {
public:
	BVH();
	~BVH();

	void buildTree(const std::vector<Triangle>& triangles);

	// return the indics of triangles
	std::vector<int> hit(const Ray& r) const;

private:
	Node* root;
};