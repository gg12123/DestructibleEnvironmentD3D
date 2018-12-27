#pragma once
#include <vector>
#include "Vector3.h"
#include "MathU.h"

class Ray
{
public:
	Ray(const Vector3& origin, const Vector3& direction)
	{
		m_Origin = origin;
		m_Direction = direction;
	}

	const auto& GetOrigin() const
	{
		return m_Origin;
	}

	const auto& GetDirection() const
	{
		return m_Direction;
	}

	bool IntersectsPlane(const Vector3& planeOrigin, const Vector3& planeNormal, Vector3& intPoint) const
	{
		if (Vector3::LinePlaneIntersection(planeOrigin, planeNormal,
			m_Origin, m_Origin + m_Direction, intPoint))
		{
			return Vector3::Dot(intPoint - m_Origin, m_Direction) >= 0.0f;
		}
		return false;
	}

private:
	Vector3 m_Origin;
	Vector3 m_Direction;
};

template<class T>
class RayCastHit
{
public:
	RayCastHit(T* hitObj, const Vector3& hitPoint)
	{
		m_HitObject = hitObj;
		m_HitPoint = hitPoint;
	}

	bool Hit() const
	{
		return m_HitObject != nullptr;
	}

	T* GetHitObject() const
	{
		return m_HitObject;
	}

	const auto& GetHitPoint() const
	{
		return m_HitPoint;
	}

private:
	T * m_HitObject;
	Vector3 m_HitPoint;
};

template<class T>
class UpdatableRayCast
{
public:
	UpdatableRayCast(const Ray& r) : m_Ray(r)
	{
		m_ClosestSqrDistToHitObj = MathU::Infinity;
		m_HitObj = nullptr;
	}

	RayCastHit<T> ToRayCastHit()
	{
		return RayCastHit<T>(m_HitObj, m_HitPoint);
	}

	template<class T1>
	void Update(const std::vector<std::unique_ptr<T1>>& objects)
	{
		Vector3 intPoint;
		for (auto& obj : objects)
		{
			if (obj->IntersectsRay(m_Ray, intPoint))
			{
				auto dist = (intPoint - m_Ray.GetOrigin()).MagnitudeSqr();

				if (dist < m_ClosestSqrDistToHitObj)
				{
					m_ClosestDistToHitObj = dist;
					m_HitObj = obj.get();
					m_HitPoint = intPoint;
				}
			}
		}
	}

private:
	float m_ClosestSqrDistToHitObj;
	T* m_HitObj;
	Vector3 m_HitPoint;
	Ray m_Ray;
};