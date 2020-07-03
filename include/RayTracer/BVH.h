#pragma once

#include <RayTracer/Triangle.h>
#include <RayTracer/Ray.h>
#include <vector>

class AABB {
public:
	AABB(const Eigen::Vector3f& min, const Eigen::Vector3f& max);
	float squareDistance(const AABB& rhs) const;
	AABB conbine(const AABB& rhs) const;
	bool hit(const Ray& r) const;
private:
	Eigen::Vector3f min;
	Eigen::Vector3f max;
};

struct Node {
	AABB aabb;
	int vertexIndex;
	Node* left;
	Node* right;

	Node(int index, const AABB& aabb);
	Node(const AABB& aabb, Node* left, Node* right);
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