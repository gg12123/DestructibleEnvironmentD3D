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

class ContactPlane
{
public:
	// The normal must point towards the associated body.
	// So is in the same direction as the impulse if there is one.
	ContactPlane(const Vector3& point, const Vector3& normal, float penetration) :
		m_Normal(normal), m_Point(point), m_Peneration(penetration)
	{
	}

	ContactPlane()
	{
	}

	auto GetPoint() const
	{
		return m_Point;
	}

	auto GetNormal() const
	{
		return m_Normal;
	}

	auto GetPeneration() const
	{
		return m_Peneration;
	}

private:
	Vector3 m_Normal;
	Vector3 m_Point;
	float m_Peneration;
};