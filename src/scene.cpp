#include "scene.h"

#include <string>


Scene::Node::Node(const std::string& n, const std::vector<math::Triangle>& tr) : name(n), triangles(tr)
{ 
	for (const auto& t : triangles)
	{
		bbox.growTo(t);
	}
	bvh.build(triangles);
}

void Scene::addNode(const std::string& name, const std::vector<math::Triangle>& triangles)
{
	nodes_.push_back({ name, triangles });
}

void Scene::addMaterial(const Material& m)
{
	materials_.push_back(m);
}

float Scene::intersect(const math::Ray& ray, float tMin, float tMax, math::Triangle& tr) const
{
	float closestT = tMax;
	float tBox;
	math::Triangle t;
	for (const auto& node : nodes_)
	{
		if (math::intersectBB(ray, node.bbox, tMin, tMax, tBox))
		{
			float dist = node.bvh.intersect(ray, tMin, closestT, t);
			if (dist < closestT)
			{
				closestT = dist;
				tr = t;
			}
		}
	}
	return closestT;
}