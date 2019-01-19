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

	Vector3 ClosestPointOnRay(const Vector3& p) const
	{

	}

private:
	Vector3 m_Origin;
	Vector3 m_Direction;
};