#pragma once

#include <algorithm>
#include <vector>
#include <cmath>
#include <cstdint>

#include "utils.h"
#include "vector.h"

template <typename T>
class BVH {
public:

    struct alignas(32) Node {
        math::BBox box;
        uint32_t leftFirst; 
        uint32_t count;     
        
        bool isLeaf() const { return count > 0; }
    };

    void build(const std::vector<T>& triangles) {
        triangles_ = triangles;
        nodes_.clear();
        nodes_.reserve(triangles.size() * 2);
        
        Node root;
        root.leftFirst = 0;
        root.count = (uint32_t)triangles_.size();
        root.box = math::BBox();
        
        
        for (const auto& t : triangles_) {
            root.box.growTo(t);
        }

        nodes_.push_back(root);
        
        split(0, 0);

        nodes_.shrink_to_fit();
    }

    float intersect(const math::Ray& ray, float tMin, float tMax, T& hitPrimitive) const {
        if (nodes_.empty()) return tMax;

        const Node* stack[64];
        const Node** stackPtr = stack;
        *stackPtr++ = &nodes_[0];

        float closestT = tMax;
        bool hitAnything = false;

        while (stackPtr != stack) {
            const Node* node = *--stackPtr;

            float tBox;
            if (!intersectBB(ray, node->box, tMin, closestT, tBox)) {
                continue;
            }
            
            if (node->isLeaf()) {

                for (uint32_t i = 0; i < node->count; ++i) {
                    float t = math::intersect(ray, triangles_[node->leftFirst + i], tMin, closestT);
                    if (t < closestT) {
                        closestT = t;
                        hitPrimitive = triangles_[node->leftFirst + i];
                        hitAnything = true;
                    }
                }
            } else {
                *stackPtr++ = &nodes_[node->leftFirst];
                *stackPtr++ = &nodes_[node->leftFirst + 1];
            }
        }

        return hitAnything ? closestT : tMax;
    }

private:
    std::vector<Node> nodes_;
    std::vector<T> triangles_;

    void split(size_t nodeIdx, int depth) {
        uint32_t first = nodes_[nodeIdx].leftFirst;
        uint32_t count = nodes_[nodeIdx].count;

        if (depth > 20 || count <= 2) {
            return;
        }
        Vector3 extent = nodes_[nodeIdx].box.size();
        int axis = 0;
        if (extent.y() > extent.x()) axis = 1;
        if (extent.z() > extent[axis]) axis = 2;

        float splitPos = nodes_[nodeIdx].box.min()[axis] + extent[axis] * 0.5f;

        auto beginIt = triangles_.begin() + first;
        auto endIt = beginIt + count;

        auto midIt = std::partition(beginIt, endIt, [axis, splitPos](const T& t) {
            return math::center(t)[axis] < splitPos;
        });

        uint32_t leftCount = (uint32_t)std::distance(beginIt, midIt);

        if (leftCount == 0 || leftCount == count) {
            return;
        }

        size_t leftChildIdx = nodes_.size();
        nodes_.emplace_back();
        nodes_.emplace_back();
        
        nodes_[leftChildIdx].leftFirst = first;
        nodes_[leftChildIdx].count = leftCount;
        nodes_[leftChildIdx].box = math::BBox();
        for (uint32_t i = 0; i < leftCount; ++i) {
            nodes_[leftChildIdx].box.growTo(triangles_[first + i]);
        }

        size_t rightChildIdx = leftChildIdx + 1;
        nodes_[rightChildIdx].leftFirst = first + leftCount;
        nodes_[rightChildIdx].count = count - leftCount;
        nodes_[rightChildIdx].box = math::BBox();
        for (uint32_t i = 0; i < nodes_[rightChildIdx].count; ++i) {
            nodes_[rightChildIdx].box.growTo(triangles_[nodes_[rightChildIdx].leftFirst + i]);
        }

        nodes_[nodeIdx].leftFirst = (uint32_t)leftChildIdx;
        nodes_[nodeIdx].count = 0;

        split(leftChildIdx, depth + 1);
        split(rightChildIdx, depth + 1);
    }
};