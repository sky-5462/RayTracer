#pragma once

#include <RayTracer/Triangle.h>
#include <RayTracer/Ray.h>
#include <vector>

// �����ĵ㣬�����м����
struct AABBTemp {
	Eigen::Vector4f min;
	Eigen::Vector4f max;
	Eigen::Vector4f center;
	int index;

	AABBTemp(int index, const Eigen::Vector4f& min, const Eigen::Vector4f& max);
};

// ���մ洢��ֻ�б߿�
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

	// ��ʼ��aabb��index
	LinearNode(const TreeNode& treeNode);
};

class BVH {
public:
	void buildTree(const std::vector<Triangle>& triangles);

	// �������е������ε������б�
	std::vector<int> hit(const Ray& r) const;

private:
	std::vector<LinearNode> linearTree;
};