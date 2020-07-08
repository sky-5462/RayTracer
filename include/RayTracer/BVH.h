#pragma once

#include <RayTracer/Triangle.h>
#include <RayTracer/Ray.h>
#include <vector>
#include <memory>

class AABB {
public:
	AABB(const Eigen::Vector3f& min, const Eigen::Vector3f& max);
	bool hit(const Ray& r) const;

	Eigen::Vector3f min;
	Eigen::Vector3f max;
	Eigen::Vector3f center;
};

struct TreeNode {
	AABB aabb;
	std::unique_ptr<TreeNode> left;
	std::unique_ptr<TreeNode> right;
	int vertexIndex;

	TreeNode(int vertexIndex, const Eigen::Vector3f& min, const Eigen::Vector3f& max);
};

class BVH {
public:
	void buildTree(const std::vector<Triangle>& triangles);

	// return the indics of triangles
	std::vector<int> hit(const Ray& r) const;

private:
	std::unique_ptr<TreeNode> root;
};