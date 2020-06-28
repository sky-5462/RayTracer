#include <RayTracer/BVH.h>

Node::Node() : vertexIndex(-1), left(nullptr), right(nullptr) {}

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

}

int BVH::hit(const Ray& r) const {
	return 0;
}
