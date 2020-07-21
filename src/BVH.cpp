#include <RayTracer/BVH.h>
#include <algorithm>
#include <stack>
#include <cfloat>
#include <memory>
#include <array>

AABBTemp::AABBTemp(int index, const Eigen::Vector4f& min, const Eigen::Vector4f& max) :
	min(min), max(max), center((min + max) * 0.5f), index(index) {
}

AABB::AABB(const Eigen::Vector4f& min, const Eigen::Vector4f& max) : min(min), max(max) {}

bool AABB::hit(const Ray& r) const {
	Eigen::Vector4f invD = r.direction.cwiseInverse();
	Eigen::Vector4f t0v = (min - r.origin).cwiseProduct(invD);
	Eigen::Vector4f t1v = (max - r.origin).cwiseProduct(invD);

	auto temp1 = _mm_load_ps(t0v.data());
	auto temp2 = _mm_load_ps(t1v.data());
	auto mask = _mm_load_ps(invD.data());
	auto t0out = _mm_blendv_ps(temp1, temp2, mask);
	auto t1out = _mm_blendv_ps(temp2, temp1, mask);

	float tmin = t0out.m128_f32[0];
	float tmax = t1out.m128_f32[0];
	for (int i = 1; i < 3; i++) {
		float t0 = t0out.m128_f32[i];
		float t1 = t1out.m128_f32[i];
		tmin = t0 > tmin ? t0 : tmin;
		tmax = t1 < tmax ? t1 : tmax;

	}
	// ƽ������ƽ�����������tmax = tmin
	if (tmax < tmin)
		return false;
	else
		return true;
}

TreeNode::TreeNode(int index, const Eigen::Vector4f& min, const Eigen::Vector4f& max) :
	vertexIndex(index), aabb(min, max) {
}

LinearNode::LinearNode(const TreeNode& treeNode) :
	vertexIndex(treeNode.vertexIndex), aabb(treeNode.aabb) {
}

void BVH::buildTree(const std::vector<Triangle>& triangles) {
	// ���������İ�Χ��
	Eigen::Vector4f min = Eigen::Vector4f::Constant(FLT_MAX);
	Eigen::Vector4f max = Eigen::Vector4f::Constant(-FLT_MAX);

	std::vector<std::unique_ptr<AABBTemp>> leafList(triangles.size());
	for (int i = 0; i < triangles.size(); ++i) {
		// һ�������εİ�Χ��
		Eigen::Vector4f tempMin = Eigen::Vector4f::Constant(FLT_MAX);
		Eigen::Vector4f tempMax = Eigen::Vector4f::Constant(-FLT_MAX);

		const auto& vertex = triangles[i].vertexPosition;
		for (int j = 0; j < 3; ++j) {
			tempMin = tempMin.cwiseMin(vertex(j));
			tempMax = tempMax.cwiseMax(vertex(j));
			min = min.cwiseMin(vertex(j));
			max = max.cwiseMax(vertex(j));
		}

		leafList[i] = std::make_unique<AABBTemp>(i, tempMin, tempMax);
	}

	// ����
	auto root = std::make_unique<TreeNode>(-1, min, max);
	std::stack<std::tuple<TreeNode*, int, int>> s;
	s.push(std::make_tuple(root.get(), 0, static_cast<int>(triangles.size())));
	do {
		auto [node, start, end] = s.top();
		s.pop();
		if (end - start <= 2) {
			// һ������ֻʣ��������ڵ�ʱ����Ȼ����һ���ڵ㣬�ҵ�������
			const auto& tempAABB = leafList[start];
			node->left = std::make_unique<TreeNode>(tempAABB->index, tempAABB->min, tempAABB->max);

			// ���ܴ��ڵڶ����ڵ㣬�ҵ�������
			if (end - start == 2) {
				const auto& tempAABB2 = leafList[start + 1];
				node->right = std::make_unique<TreeNode>(tempAABB2->index, tempAABB2->min, tempAABB2->max);
			}
		}
		else {
			// ��ÿһ�λ��֣���ѡ���Χ���������
			const auto& aabb = node->aabb;
			Eigen::Vector4f diff = aabb.max - aabb.min;
			float axisLength = diff(0);
			int selectedAxis = 0;
			for (int i = 1; i < 3; ++i) {
				if (diff(i) > axisLength) {
					axisLength = diff(i);
					selectedAxis = i;
				}
			}

			// ���ڵ㰴������λ����ѡ�����ϵ�˳������
			std::sort(leafList.begin() + start, leafList.begin() + end,
					  [selectedAxis](const auto& lhs, const auto& rhs) {
						  return lhs->center(selectedAxis) < rhs->center(selectedAxis);
					  });

			// �ٽ�������Ľڵ�԰�֣���Ϊ��������
			int splitStart = (start + end + 1) / 2;

			// �ҵ�ǰһ�����ֵİ�Χ��
			min = Eigen::Vector4f::Constant(FLT_MAX);
			max = Eigen::Vector4f::Constant(-FLT_MAX);
			for (int i = start; i < splitStart; ++i) {
				int vertexIndex = leafList[i]->index;
				const auto& vertex = triangles[vertexIndex].vertexPosition;
				for (int j = 0; j < 3; ++j) {
					min = min.cwiseMin(vertex(j));
					max = max.cwiseMax(vertex(j));
				}
			}
			node->left = std::make_unique<TreeNode>(-1, min, max);
			s.push(std::make_tuple(node->left.get(), start, splitStart));

			// �ҵ���һ�����ֵİ�Χ��
			min = Eigen::Vector4f::Constant(FLT_MAX);
			max = Eigen::Vector4f::Constant(-FLT_MAX);
			for (int i = splitStart; i < end; ++i) {
				int vertexIndex = leafList[i]->index;
				const auto& vertex = triangles[vertexIndex].vertexPosition;
				for (int j = 0; j < 3; ++j) {
					min = min.cwiseMin(vertex(j));
					max = max.cwiseMax(vertex(j));
				}
			}
			node->right = std::make_unique<TreeNode>(-1, min, max);
			s.push(std::make_tuple(node->right.get(), splitStart, end));
		}
	} while (!s.empty());

	// ת����������
	std::vector<TreeNode*> ptrQueue;
	ptrQueue.push_back(root.get());
	linearTree.emplace_back(*root);
	for (int i = 0; i < ptrQueue.size(); ++i) {
		auto nodePtr = ptrQueue[i];
		if (nodePtr->left) {
			ptrQueue.push_back(nodePtr->left.get());
			linearTree[i].left = ptrQueue.size() - 1;
			linearTree.emplace_back(*(nodePtr->left));
		}
		if (nodePtr->right) {
			ptrQueue.push_back(nodePtr->right.get());
			linearTree[i].right = ptrQueue.size() - 1;
			linearTree.emplace_back(*(nodePtr->right));
		}
	}
}

std::vector<int> BVH::hit(const Ray& r) const {
	// Ԥ���ռ䣬���ٿռ���俪��
	std::vector<int> result;
	result.reserve(linearTree.size() / 4);

	// ջ�ռ����ݹ�ջ��32���㹻���ڸ��ڵ���
	std::array<int, 32> stack = { 0 };
	int stackSize = 1;
	do {
		int nodeIndex = stack[stackSize - 1];
		stackSize--;
		const auto& node = linearTree[nodeIndex];
		bool hasHit = node.aabb.hit(r);
		if (hasHit) {
			if (node.vertexIndex >= 0)
				result.push_back(node.vertexIndex);
			else {
				if (node.left > 0) {
					stack[stackSize] = node.left;
					stackSize++;
				}
				if (node.right > 0) {
					stack[stackSize] = node.right;
					stackSize++;
				}
			}
		}
	} while (stackSize != 0);

	return result;
}
