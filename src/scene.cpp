#include "scene.h"

#include <string>


void Scene::Node::rebuild()
{
    bbox = math::BBox();
    for (const auto& t : triangles)
    {
        bbox.growTo(t);
    }
    bvh.build(triangles);
}

Scene::Node::Node(const std::string& name, size_t matIdx, const std::vector<math::Triangle>& tr) : name(name), matIndex(matIdx), triangles(tr)
{ 
	for (const auto& t : triangles)
	{
		bbox.growTo(t);
	}
	bvh.build(triangles);
}

void Scene::addNode(const std::string& name, const std::vector<math::Triangle>& triangles, size_t matIndex)
{	
	nodes_.emplace_back(name, matIndex, triangles);
}

void Scene::addMaterial(const Material& m)
{
	materials_.push_back(m);
}

std::pair<const Scene::Node*, float> Scene::intersect(const math::Ray& ray, float tMin, float tMax, math::Triangle& tr) const
{
	float closestT = tMax;
	float tBox;
	math::Triangle t;
	const Node* node = nullptr;
	for (const auto& n : nodes_)
	{
		if (math::intersectBB(ray, n.bbox, tMin, tMax, tBox))
		{
			float dist = n.bvh.intersect(ray, tMin, closestT, t);
			if (dist < closestT)
			{
				closestT = dist;
				tr = t;
				node = &n;
			}
		}
	}
	return {node, closestT};
}