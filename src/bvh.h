#pragma once

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

#include "utils.h"
#include "vector.h"

template <typename T> class BVH {
public:
  void build(const std::vector<T> &shapes) {
    root_ = std::make_unique<Node>();

    for (const auto &t : shapes) {
      root_->box.growTo(t);
    }

    root_->shapes = shapes;

    split(*root_.get());
  }

  float intersect(const math::Ray &ray, float tMin, float tMax, T &tr) const {
    return intersect(ray, *root_.get(), tMin, tMax, tr);
  }

  void print() const {
    if (!root_) {
      std::cout << "BVH is empty.\n";
      return;
    }

    int emptyNodes = 0;
    int heavyNodes = 0;

    printNode(*root_, 0, emptyNodes, heavyNodes);

    std::cout << "--------------------------------\n";
    std::cout << "BVH Statistics:\n";
    std::cout << "Total Empty Nodes (0 tris): " << emptyNodes << "\n";
    std::cout << "Total Heavy Nodes (>8 tris): " << heavyNodes << "\n";
    std::cout << "--------------------------------\n";
  }

private:
  struct Node {
    math::BBox box;
    std::vector<T> shapes;
    std::unique_ptr<Node> childA;
    std::unique_ptr<Node> childB;
  };

  void split(Node &parent, int depth = 0) const {
    if (depth > 10)
      return;
    if (parent.shapes.size() <= 2)
      return;

    Vector3 size = parent.box.size();

    int splitAxis = size.x() > std::max(size.y(), size.z()) ? 0
                    : size.y() > size.z()                   ? 1
                                                            : 2;

    float splitPos = 0.0f;
    for (const auto &tr : parent.shapes)
      splitPos += math::center(tr)[splitAxis];
    splitPos /= parent.shapes.size();

    auto childA = std::make_unique<Node>();
    auto childB = std::make_unique<Node>();

    for (const auto &tr : parent.shapes) {
      float c = math::center(tr)[splitAxis];
      Node *dst = (c < splitPos) ? childA.get() : childB.get();

      dst->shapes.push_back(tr);
      dst->box.growTo(tr);
    }

    if (childA->shapes.empty() || childB->shapes.empty()) {
      return;
    }

    parent.childA = std::move(childA);
    parent.childB = std::move(childB);

    parent.shapes = {};

    split(*parent.childA, depth + 1);
    split(*parent.childB, depth + 1);
  }

  void printNode(const Node &node, int depth, int &emptyCount,
                 int &heavyCount) const {

    if (node.shapes.empty()) {
      emptyCount++;
    }
    if (node.shapes.size() > 8) {
      heavyCount++;
    }

    for (int i = 0; i < depth; i++)
      std::cout << "  ";

    const Vector3 size = node.box.size();
    const Vector3 center = node.box.center();

    std::cout << "Node(depth=" << depth << ", tris=" << node.shapes.size()
              << ", center=[" << center.x() << ", " << center.y() << ", "
              << center.z() << "]"
              << ", size=[" << size.x() << ", " << size.y() << ", " << size.z()
              << "])";

    if (node.shapes.size() > 8)
      std::cout << " <--- HEAVY";
    if (node.shapes.empty() && !node.childA && !node.childB)
      std::cout << " <--- USELESS LEAF";

    std::cout << "\n";

    if (node.childA)
      printNode(*node.childA, depth + 1, emptyCount, heavyCount);

    if (node.childB)
      printNode(*node.childB, depth + 1, emptyCount, heavyCount);
  }

  float intersect(const math::Ray &ray, const Node &node, float tMin,
                  float tMax, T &tr) const {
    float tBox;
    if (!intersectBB(ray, node.box, tMin, tMax, tBox))
      return tMax;

    if (node.shapes.empty()) {
      // Internal Node (or empty leaf)
      if (!node.childA && !node.childB)
        return tMax;

      // Ensure children exist before dereferencing
      T trA;
      T trB;

      float tHitA = tMax;
      if (node.childA)
        tHitA = intersect(ray, *node.childA, tMin, tMax, trA);

      float searchMaxB = std::min(tMax, tHitA);
      float tHitB = tMax;
      if (node.childB)
        tHitB = intersect(ray, *node.childB, tMin, searchMaxB, trB);

      if (tHitB < tHitA) {
        tr = trB;
        return tHitB;
      } else {
        tr = trA;
        return tHitA;
      }
    } else {
      // Leaf Node with shapes
      float closestT = tMax;
      for (const auto &triangle : node.shapes) {
        float t = math::intersect(ray, triangle, tMin, closestT);
        if (t < closestT) {
          closestT = t;
          tr = triangle;
        }
      }
      return closestT;
    }
  }

  std::unique_ptr<Node> root_;
};
