#pragma once

#include <RayTracer/Triangle.h>
#include <RayTracer/Ray.h>
#include <vector>
#include <memory>

// with center, used for intermediate storage
struct AABBTemp {
	Eigen::Vector4f min;
	Eigen::Vector4f max;
	Eigen::Vector4f center;
	int index;

	AABBTemp(int index, const Eigen::Vector4f& min, const Eigen::Vector4f& max);
};

// only have min and max, used for final storage and hit checking
struct AABB {
	Eigen::Vector4f min;
	Eigen::Vector4f max;

	AABB(const Eigen::Vector4f& min, const Eigen::Vector4f& max);
	bool hit(const Ray& r) const;
};

struct TreeNode {
	AABB aabb;
	std::unique_ptr<TreeNode> left;
	std::unique_ptr<TreeNode> right;
	int vertexIndex;

	TreeNode(int vertexIndex, const Eigen::Vector4f& min, const Eigen::Vector4f& max);
};

class BVH {
public:
	void buildTree(const std::vector<Triangle>& triangles);

	// return the indics of triangles
	std::vector<int> hit(const Ray& r) const;

private:
	std::unique_ptr<TreeNode> root;
};