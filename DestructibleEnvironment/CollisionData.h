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
	ContactPlane(const Vector3& point, const Vector3& normal) : m_Normal(normal), m_Point(point)
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

private:
	Vector3 m_Normal;
	Vector3 m_Point;
};