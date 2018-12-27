#pragma once
#include "Vector3.h"

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