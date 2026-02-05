#pragma once

#include "bvh.h"
#include "vector.h"

#include <vector>

struct Material {
  Vector3 albedo;
  Vector3 emission;
  float metallic;
  float roughness;
};

struct Camera {
  Vector3 pos;
  Vector3 target;
  Vector3 up;
  float fov;
  float aspectRatio;
};

class Scene {
  struct Node {
    Node(const std::string &n, const std::vector<math::Triangle> &tr);
    std::string name;
    math::BBox bbox;
    std::vector<math::Triangle> triangles;

    BVH<math::Triangle> bvh;
  };

public:
  void addNode(const std::string &name,
               const std::vector<math::Triangle> &triangles);
  void addMaterial(const Material &m);
  const std::vector<Material> &materials() const { return materials_; }

  float intersect(const math::Ray &ray, float tMin, float tMax,
                  math::Triangle &tr) const;

  void setCamera(const Camera &camera) { camera_ = camera; }
  const Camera &camera() const { return camera_; }

private:
  Camera camera_;
  std::vector<Node> nodes_;
  std::vector<Material> materials_;
};