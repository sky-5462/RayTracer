#pragma once

#include <RayTracer/Triangle.h>
#include <RayTracer/Ray.h>
#include <vector>

// 有中心点，用在中间过程
struct AABBTemp {
	Eigen::Vector4f min;
	Eigen::Vector4f max;
	Eigen::Vector4f center;
	int index;

	AABBTemp(int index, const Eigen::Vector4f& min, const Eigen::Vector4f& max);
};

// 最终存储，只有边框
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

struct LinearNode {
	AABB aabb;
	int left;
	int right;
	int vertexIndex;

	// 初始化aabb和index
	LinearNode(const TreeNode& treeNode);
};

class BVH {
public:
	void buildTree(const std::vector<Triangle>& triangles);

	// 返回命中的三角形的索引列表
	std::vector<int> hit(const Ray& r) const;

private:
	std::vector<LinearNode> linearTree;
};