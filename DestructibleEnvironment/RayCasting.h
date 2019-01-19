#pragma once
#include <vector>
#include "Vector3.h"
#include "MathU.h"
#include "Ray.h"

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
		return m_HitObject;
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
					m_ClosestSqrDistToHitObj = dist;
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