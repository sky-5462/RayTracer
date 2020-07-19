#include <RayTracer/BVH.h>
#include <algorithm>
#include <stack>
#include <cfloat>

AABBTemp::AABBTemp(int index, const Eigen::Vector4f& min, const Eigen::Vector4f& max) :
	min(min), max(max), center((min + max) * 0.5f), index(index) {
}

AABB::AABB(const Eigen::Vector4f& min, const Eigen::Vector4f& max) : min(min), max(max) {}

bool AABB::hit(const Ray& r) const {
	Eigen::Vector4f invD = r.direction.cwiseInverse();
	Eigen::Vector4f t0v = (min - r.origin).cwiseProduct(invD);
	Eigen::Vector4f t1v = (max - r.origin).cwiseProduct(invD);
	float tmin = 0.0f, tmax = FLT_MAX;
	for (int i = 0; i < 3; i++) {
		float t0 = t0v(i);
		float t1 = t1v(i);
		if (invD(i) < 0.0f)
			std::swap(t0, t1);
		tmin = fmaxf(t0, tmin);
		tmax = fminf(t1, tmax);

		// ƽ������ƽ�����������tmax = tmin
		if (tmax < tmin)
			return false;
	}
	return true;
}

TreeNode::TreeNode(int index, const Eigen::Vector4f& min, const Eigen::Vector4f& max) :
	vertexIndex(index), aabb(min, max) {
}

void BVH::buildTree(const std::vector<Triangle>& triangles) {
	if (triangles.empty())
		return;

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
	root = std::make_unique<TreeNode>(-1, min, max);
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
