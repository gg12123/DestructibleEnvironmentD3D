#pragma once
#include "Vector3.h"
#include "Plane.h"

class Impulse
{
public:
	Impulse(const Vector3& worldImpulse, const Vector3& worldCollPoint, float impact) : 
		WorldImpulse(worldImpulse),
		WorldImpulsePoint(worldCollPoint),
		Impact(impact)
	{
	}

	Vector3 WorldImpulse;
	Vector3 WorldImpulsePoint;
	float Impact;
};

class ContactManifold
{
public:
	// The normal must point towards the associated body.
	// So is in the same direction as the impulse if there is one.
	ContactManifold(const Vector3& point, const Vector3& normal) : m_Plane(normal, point), m_Point(point)
	{
	}

	ContactManifold()
	{
	}

	auto GetPoint() const
	{
		return m_Plane.GetP0();
	}

	auto GetNormal() const
	{
		return m_Plane.GetNormal();
	}

private:
	Plane m_Plane;
	Vector3 m_Point;
};