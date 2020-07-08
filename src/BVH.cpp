#include <RayTracer/BVH.h>
#include <algorithm>
#include <stack>

AABB::AABB(const Eigen::Vector3f& min, const Eigen::Vector3f& max) :
	min(min), max(max), center((min + max) * 0.5f) {
}

bool AABB::hit(const Ray& r) const {
	float tmin = 0.0f, tmax = FLT_MAX;
	for (int i = 0; i < 3; i++) {
		float invD = 1.0f / r.direction(i);
		float t0 = (min(i) - r.origin(i)) * invD;
		float t1 = (max(i) - r.origin(i)) * invD;
		if (invD < 0.0f)
			std::swap(t0, t1);
		tmin = fmaxf(t0, tmin);
		tmax = fminf(t1, tmax);
		// a triangle paralleled to one coordinate plane means tmax = tmin
		if (tmax < tmin)
			return false;
	}
	return true;
}

TreeNode::TreeNode(int index, const Eigen::Vector3f& min, const Eigen::Vector3f& max) :
	vertexIndex(index), aabb(min, max) {
}

void BVH::buildTree(const std::vector<Triangle>& triangles) {
	if (triangles.empty())
		return;

	// for all triangles
	Eigen::Vector3f min(FLT_MAX, FLT_MAX, FLT_MAX);
	Eigen::Vector3f max(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	// leaf nodes
	std::vector<std::unique_ptr<TreeNode>> leafList(triangles.size());
	for (int i = 0; i < triangles.size(); ++i) {
		// for one triangle
		Eigen::Vector3f tempMin(FLT_MAX, FLT_MAX, FLT_MAX);
		Eigen::Vector3f tempMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);

		const auto& vertex = triangles[i].vertex;
		for (int j = 0; j < 3; ++j) {
			tempMin = tempMin.cwiseMin(vertex(j));
			tempMax = tempMax.cwiseMax(vertex(j));
			min = min.cwiseMin(vertex(j));
			max = max.cwiseMax(vertex(j));
		}

		leafList[i] = std::make_unique<TreeNode>(i, tempMin, tempMax);
	}

	// build tree
	root = std::make_unique<TreeNode>(-1, min, max);
	std::stack<std::tuple<TreeNode*, int, int>> s;
	s.push(std::make_tuple(root.get(), 0, static_cast<int>(triangles.size())));
	do {
		auto [node, start, end] = s.top();
		s.pop();
		if (end - start <= 2) {
			// must have one element
			node->left = std::move(leafList[start]);
			// may have two
			if (end - start == 2) {
				node->right = std::move(leafList[start + 1]);
			}
		}
		else {
			// find the longest axis
			const auto& aabb = node->aabb;
			float axisLength = aabb.max(0) - aabb.min(0);
			int selectedAxis = 0;
			for (int i = 1; i < 3; ++i) {
				if (aabb.max(i) - aabb.min(i) > axisLength) {
					axisLength = aabb.max(i) - aabb.min(i);
					selectedAxis = i;
				}
			}

			// sort those elements in the range by the selected axis
			std::sort(leafList.begin() + start, leafList.begin() + end,
					  [selectedAxis](const auto& lhs, const auto& rhs) {
						  return lhs->aabb.center(selectedAxis) < rhs->aabb.center(selectedAxis);
					  });

			// split in halt
			int splitStart = (start + end + 1) / 2;

			// find aabb for left part
			min = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
			max = Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			for (int i = start; i < splitStart; ++i) {
				int index = leafList[i]->vertexIndex;
				const auto& vertex = triangles[index].vertex;
				for (int j = 0; j < 3; ++j) {
					min = min.cwiseMin(vertex(j));
					max = max.cwiseMax(vertex(j));
				}
			}
			node->left = std::make_unique<TreeNode>(-1, min, max);
			s.push(std::make_tuple(node->left.get(), start, splitStart));

			// find aabb for right part
			min = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
			max = Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			for (int i = splitStart; i < end; ++i) {
				int index = leafList[i]->vertexIndex;
				const auto& vertex = triangles[index].vertex;
				for (int j = 0; j < 3; ++j) {
					min = min.cwiseMin(vertex(j));
					max = max.cwiseMax(vertex(j));
				}
			}
			node->right = std::make_unique<TreeNode>(-1, min, max);
			s.push(std::make_tuple(node->right.get(), splitStart, end));
		}
	} while (!s.empty());
}

std::vector<int> BVH::hit(const Ray& r) const {
	std::vector<int> result;
	if (!root)
		return result;

	std::stack<TreeNode*> s;
	s.push(root.get());
	do {
		auto node = s.top();
		s.pop();
		bool hasHit = node->aabb.hit(r);
		if (hasHit) {
			if (node->vertexIndex >= 0)
				result.push_back(node->vertexIndex);
			else {
				if (node->left)
					s.push(node->left.get());
				if (node->right)
					s.push(node->right.get());
			}
		}
	} while (!s.empty());

	return result;
}
