#include "scene.h"

#include <string>


void Scene::Node::rebuild()
{
    bbox = math::BBox();
	auto pr = bvh.primitives();
    for (const auto& t : pr)
    {
        bbox.growTo(t);
    }
    bvh.build(pr);
}

Scene::Node::Node(const std::string& name, size_t matIdx, const std::vector<math::Triangle>& triangles) : name(name), matIndex(matIdx)
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

std::tuple<float, math::Triangle, Material> Scene::intersect(const math::Ray& ray, float tMin, float tMax) const
{
	float closestT = tMax;
	float tBox;
	math::Triangle t;
	size_t matIndex = 0;
	for (const auto& n : nodes_)
	{
		if (math::intersectBB(ray, n.bbox, tMin, closestT, tBox))
		{
			float dist = n.bvh.intersect(ray, tMin, closestT, t);
			if (dist < closestT)
			{
				closestT = dist;
				matIndex = n.matIndex;
			}
		}
	}
	return {closestT, t, materials_[matIndex]};
}