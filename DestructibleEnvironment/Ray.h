#pragma once
#include "Vector3.h"

class Ray
{
public:
	Ray()
	{
	}

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

	Vector3 ClosestPointOnRayAsInfLine(const Vector3& c) const
	{
		auto a = m_Origin;
		auto b = m_Origin + m_Direction;

		auto ab = b - a;
		auto t = Vector3::Dot(c - a, ab) / Vector3::Dot(ab, ab);

		return a + t * ab;
	}

private:
	Vector3 m_Origin;
	Vector3 m_Direction;
};