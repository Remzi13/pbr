#include "utils.h"

#include <random>

namespace math {

	
	Vector3 center(const Triangle& triangle)
	{
		return (triangle.a + triangle.b + triangle.c) / 3.0f;
	}

	Vector3 center(const Sphere& sp)
	{
		return sp.pos;
	}

	float intersectPlane2(const math::Ray& ray, const Vector3& normal, float d, float tMin, float tMax)
	{
		const float dist = dot(normal, ray.origin) - d;
		const float dotND = dot(ray.direction, normal);
		if (dotND == 0.0)
		{
			if (dist == 0.0 && tMin == 0.0)
			{
				return 0.0;
			}
			return tMax;
		}
		const float t = dist / -dotND;
		if (t< tMin || t > tMax)
			return tMax;
		return t;
	}

	float intersect(const math::Ray& ray, const Triangle& tr, float tMin, float tMax)
	{
		const Vector3 normal = unit_vector(cross(tr.b - tr.a, tr.c - tr.a));
		const float d = dot(normal, tr.a);
		const float t = intersectPlane2(ray, normal, d, tMin, tMax);
		if (t == tMax)
			return tMax;
		const Vector3 p = ray.origin + ray.direction * t;
		if (dot(cross(tr.b - tr.a, p - tr.a), normal) < 0.0f)
			return tMax;
		if (dot(cross(tr.c - tr.b, p - tr.b), normal) < 0.0f)
			return tMax;
		if (dot(cross(tr.a - tr.c, p - tr.c), normal) < 0.0f)
			return tMax;
		return t;
	}

	float intersect(const math::Ray& ray, const Sphere& sp, float tMin, float tMax)
	{
		const Vector3 origin = ray.origin - sp.pos;
		const float A = 1;
		const float B = 2.0f * dot(origin, ray.direction);
		const float C = dot(origin, origin) - sp.radius * sp.radius;
		const float D = B * B - 4 * A * C;
		if (D < 0.0f)
			return tMax;
		const float sqrtD = std::sqrt(D);
		const float t0 = (-B - sqrtD) / (2.0f * A);
		if (t0 >= tMin && t0 < tMax) return t0;
		const float t1 = (-B + sqrtD) / (2.0f * A);
		if (t1 >= tMin && t1 < tMax) return t1;
		return tMax;
	}


	BBox::BBox()
	{
		float inf = std::numeric_limits<float>::max();
		min_ = Vector3(inf, inf, inf);
		max_ = Vector3(-inf, -inf, -inf);
	}

	Vector3 BBox::center() const
	{
		return (min_ + max_) * 0.5f;
	}

	Vector3 BBox::size() const
	{
		return max_ - min_;
	}

	void BBox::growTo(const Vector3& point)
	{
		min_ = ::min(min_, point);
		max_ = ::max(max_, point);
	}

	void BBox::growTo(const math::Triangle& t)
	{
		growTo(t.a);
		growTo(t.b);
		growTo(t.c);
	}

	void BBox::growTo(const math::Sphere& sp)
	{

	}


	bool intersectBB(const math::Ray& ray, const BBox& box, float tMin, float tMax, float& tHit)
	{
		for (int i = 0; i < 3; i++)
		{
			const float origin = ray.origin[i];
			const float dir = ray.direction[i];
			const float minB = box.min()[i];
			const float maxB = box.max()[i];

			if (fabs(dir) < 1e-8f)
			{
				if (origin < minB || origin > maxB)
					return false;
				continue;
			}

			const float invD = 1.0f / dir;
			float t0 = (minB - origin) * invD;
			float t1 = (maxB - origin) * invD;

			if (t0 > t1) std::swap(t0, t1);

			tMin = std::max(tMin, t0);
			tMax = std::min(tMax, t1);

			if (tMax < tMin)
				return false;
		}

		tHit = tMin;
		return true;
	}
}

float randomFloat()
{
	thread_local std::uniform_real_distribution<float> distribution(0.0, 1.0);
	thread_local std::mt19937 generator;
	return distribution(generator);
}

float randFloat(float min, float max)
{
	return min + (max - min) * (randomFloat());
}

Vector3 randVector(float min, float max)
{
	return Vector3(randFloat(min, max), randFloat(min, max), randFloat(min, max));
}

Vector3 randUnitVector()
{
	while (true)
	{
		Vector3 p = randVector(-1, 1);
		float l = p.length_squared();
		if (1e-160 < l && l <= 1)
			return p / std::sqrt(l);
	}
}
