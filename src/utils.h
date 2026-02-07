#pragma once

#include "vector.h"

constexpr float PI = 3.14159265359f;
constexpr float INV_PI = 1.0f / PI;
constexpr float EPS = 0.00000001f;

namespace math {

	struct Triangle
	{
		Vector3 a;
		Vector3 b;
		Vector3 c;
		Vector3 na;
		Vector3 nb;
		Vector3 nc;
		size_t matIndex;
	};

	struct Sphere
	{
		Vector3 pos;
		float radius;
		int matIndex;
	};


	Vector3 center(const Triangle& triangle);
	Vector3 center(const Sphere& sp);

	struct Ray
	{
		Vector3 origin;
		Vector3 direction;
	};

	class BBox
	{
	public:
		BBox();

		const Vector3& min() const { return min_; }
		const Vector3& max() const { return max_; }

		void growTo(const Vector3& point);
		void growTo(const math::Triangle& t);
		void growTo(const math::Sphere& t);

		Vector3 center() const;
		Vector3 size() const;

	private:
		Vector3 min_;
		Vector3 max_;
	};



	template <typename T>
	constexpr T saturate(T x) {
		return std::max(T(0.0), std::min(x, T(1.0)));
	}

	template <typename T, typename U>
	constexpr T lerp(const T& a, const T& b, const U& t) {
		return a + (b - a) * t;
	}

	float intersectPlane2(const Ray& ray, const Vector3& normal, float d, float tMin, float tMax);
	float intersect(const Ray& ray, const Triangle& tr, float tMin, float tMax);
	float intersect(const Ray& ray, const Sphere& sp, float tMin, float tMax);
	bool intersectBB(const Ray& ray, const BBox& box, float tMin, float tMax, float& tHit);
}
float randomFloat();
float randFloat(float min, float max);
Vector3 randUnitVector();
Vector3 randVector(float min, float max);