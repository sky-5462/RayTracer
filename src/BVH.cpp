#include <RayTracer/BVH.h>
#include <list>

AABB::AABB(const Eigen::Vector3f& min, const Eigen::Vector3f& max) : min(min), max(max) {}

float AABB::squareDistance(const AABB& rhs) const {
	Eigen::Vector3f leftCenter = (min + max) / 2.0f;
	Eigen::Vector3f rightCenter = (rhs.min + rhs.max) / 2.0f;
	Eigen::Vector3f diff = leftCenter - rightCenter;
	auto squareDist = diff.dot(diff);
	return squareDist;
}

AABB AABB::conbine(const AABB& rhs) const {
	auto bigger = *this;
	for (int j = 0; j < 3; ++j) {
		bigger.min(j) = fminf(bigger.min(j), rhs.min(j));
		bigger.max(j) = fmaxf(bigger.max(j), rhs.max(j));
	}
	return bigger;
}

bool AABB::hit(const Ray& r) const {
	auto tmin = 0.0f, tmax = FLT_MAX;
	for (int i = 0; i < 3; i++) {
		auto invD = 1.0f / r.direction(i);
		auto t0 = (min(i) - r.origin(i)) * invD;
		auto t1 = (max(i) - r.origin(i)) * invD;
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

Node::Node(int index, const AABB& aabb) : vertexIndex(index), aabb(aabb), left(nullptr), right(nullptr) {}

Node::Node(const AABB& aabb, Node* left, Node* right) : vertexIndex(-1), aabb(aabb), left(left), right(right) {}

Node::~Node() {
	if (left != nullptr)
		delete left;
	if (right != nullptr)
		delete right;
}

BVH::BVH() : root(nullptr) {}

BVH::~BVH() {
	if (root != nullptr)
		delete root;
}

void BVH::buildTree(const std::vector<Triangle>& triangles) {
	// leaf nodes
	std::list<Node*> initialList;
	for (int i = 0; i < triangles.size(); ++i) {
		auto& vertex = triangles[i].vertex;
		Eigen::Vector3f min = vertex(0);
		Eigen::Vector3f max = vertex(0);
		// vertex
		for (int j = 1; j < 3; ++j) {
			// coordinate
			for (int k = 0; k < 3; ++k) {
				auto num = vertex(j)(k);
				min(k) = fminf(min(k), num);
				max(k) = fmaxf(max(k), num);
			}
		}

		initialList.push_back(new Node(i, AABB(min, max)));
	}

	// each iteration connect two nearest nodes
	for (int i = 0; i < triangles.size() - 1; ++i) {
		auto minLeftIt = initialList.begin();
		auto minRightIt = minLeftIt;
		auto min = FLT_MAX;

		// search the nearest nodes
		auto leftIt = minLeftIt;
		for (int j = 0; j < initialList.size() - 1; ++j) {
			auto rightIt = leftIt;
			++rightIt;
			do {
				auto squareDist = (*leftIt)->aabb.squareDistance((*rightIt)->aabb);
				if (squareDist < min) {
					min = squareDist;
					minLeftIt = leftIt;
					minRightIt = rightIt;
				}
				++rightIt;
			} while (rightIt != initialList.end());
		}

		// combine the two nodes with a bigger bounding box
		auto bigger = (*minLeftIt)->aabb.conbine((*minRightIt)->aabb);

		auto newNode = new Node(bigger, *minLeftIt, *minRightIt);
		*minLeftIt = newNode;
		initialList.erase(minRightIt);
	}

	root = initialList.front();
}

void recursiveCheck(std::vector<int>& result, const Node* node, const Ray& r) {
	auto hasHit = node->aabb.hit(r);
	if (hasHit) {
		if (node->left == nullptr && node->right == nullptr) {
			result.push_back(node->vertexIndex);
		}
		else {
			if (node->left != nullptr)
				recursiveCheck(result, node->left, r);
			if (node->right != nullptr)
				recursiveCheck(result, node->right, r);
		}
	}
}

std::vector<int> BVH::hit(const Ray& r) const {
	std::vector<int> result;
	if (root == nullptr)
		return result;

	recursiveCheck(result, root, r);
	return result;
}
