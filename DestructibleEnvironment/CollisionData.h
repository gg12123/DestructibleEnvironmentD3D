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

	const Vector3 WorldImpulse;
	const Vector3 WorldImpulsePoint;

	const float Impact;
};

class ContactManifold
{
public:
	// The normal must point towards the associated body.
	// So is in the same direction as the impulse if there is one.
	ContactManifold(const Vector3& worldPoint, const Vector3& worldNormal) : m_Plane(worldNormal, worldPoint)
	{
	}

	auto GetWorldPoint() const
	{
		return m_Plane.GetP0();
	}

	auto GetWorldNormal() const
	{
		return m_Plane.GetNormal();
	}

private:
	Plane m_Plane;
};