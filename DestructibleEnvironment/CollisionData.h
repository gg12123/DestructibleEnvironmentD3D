#pragma once
#include "Vector3.h"
#include "Plane.h"

class Impulse
{
public:
	Impulse()
	{
	}

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
	ContactPlane(float contactMin, float contactMax, const Vector3& normal) :
		m_Normal(normal), m_ContactMin(contactMin), m_ContactMax(contactMax)
	{
	}

	ContactPlane()
	{
	}

	auto GetNormal() const
	{
		return m_Normal;
	}

	auto GetPeneration() const
	{
		return m_ContactMax - m_ContactMin;
	}

	auto GetContactMin() const
	{
		return m_ContactMin;
	}

	auto GetContactMax() const
	{
		return m_ContactMax;
	}

private:
	Vector3 m_Normal;
	Vector3 m_Point;
	float m_ContactMin;
	float m_ContactMax;
};