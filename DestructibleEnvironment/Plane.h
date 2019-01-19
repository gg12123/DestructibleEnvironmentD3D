#pragma once
#include "Vector3.h"
#include "Ray.h"

class Plane
{
public:
	Plane()
	{
	}

	Plane(const Vector3& n, const Vector3& p0) : m_Normal(n), m_P0(p0)
	{
	}

	bool Intersects(const Plane& other, Ray& inter) const
	{

	}

	bool Intersects(const Plane& otherA, const Plane& otherB, Vector3& inter) const
	{

	}

	const auto& GetNormal() const
	{
		return m_Normal;
	}

	const auto& GetP0() const
	{
		return m_P0;
	}

private:
	Vector3 m_Normal;
	Vector3 m_P0;
};
