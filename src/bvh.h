#pragma once

#include "vector.h"
#include "utils.h"

#include <vector>


template<typename T>
class BVH
{
public:

	void build(const std::vector<T>& shapes)
	{ 
		root_ = std::make_unique<Node>();
	
		for (const auto& t : shapes)
		{
			root_->box.growTo(t);
		}
	
		root_->shapes = shapes;
	
		split(*root_.get());
	}

	float intersect(const math::Ray& ray, float tMin, float tMax, T& tr) const
	{
		return intersect(ray, *root_.get(), tMin, tMax, tr);
	}

	void print() const;
	
private:
	struct Node
	{
		math::BBox box;
		std::vector<T> shapes;
		std::unique_ptr<Node> childA;
		std::unique_ptr<Node> childB;
	};
	
	void split(Node& parent, int depth = 0) const
	{
		if (depth > 10) return;
		if (parent.shapes.size() <= 2) return;

		Vector3 size = parent.box.size();

		int splitAxis =
			size.x() > std::max(size.y(), size.z()) ? 0 :
			size.y() > size.z() ? 1 : 2;


		float splitPos = 0.0f;
		for (const auto& tr : parent.shapes)
			splitPos += math::center(tr)[splitAxis];
		splitPos /= parent.shapes.size();

		auto childA = std::make_unique<Node>();
		auto childB = std::make_unique<Node>();

		for (const auto& tr : parent.shapes)
		{
			float c = math::center(tr)[splitAxis];
			Node* dst = (c < splitPos) ? childA.get() : childB.get();

			dst->shapes.push_back(tr);
			dst->box.growTo(tr);
		}

		if (childA->shapes.empty() || childB->shapes.empty())
		{
			return;
		}

		parent.childA = std::move(childA);
		parent.childB = std::move(childB);

		parent.shapes = {};

		split(*parent.childA, depth + 1);
		split(*parent.childB, depth + 1);
	}

	void printNode(const Node& node, int depth, int& emptyCount, int& heavyCount) const;

	float intersect(const math::Ray& ray, const Node& node, float tMin, float tMax, T& tr) const
	{
		float tBox;
		if (!intersectBB(ray, node.box, tMin, tMax, tBox))
			return tMax;

		if (node.shapes.empty())
		{
			T trA;
			T trB;

			float tHitA = intersect(ray, *node.childA, tMin, tMax, trA);

			float searchMaxB = std::min(tMax, tHitA);
			float tHitB = intersect(ray, *node.childB, tMin, searchMaxB, trB);

			if (tHitB < tHitA) {
				tr = trB;
				return tHitB;
			}
			else {
				tr = trA;
				return tHitA;
			}
		}
		else
		{
			float closestT = tMax;
			for (const auto& triangle : node.shapes)
			{
				float t = math::intersect(ray, triangle, tMin, closestT);
				if (t < closestT)
				{
					closestT = t;
					tr = triangle;
				}
			}
			return closestT;
		}
	}

	std::unique_ptr<Node> root_;
};

