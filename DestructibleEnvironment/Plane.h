#pragma once
#include "Vector3.h"
#include "Ray.h"

class Plane
{
public:
	Plane()
	{
	}

	Plane(const Vector3& n, const Vector3& p0) : m_Normal(n), m_D(Vector3::Dot(p0, n))
	{
	}

	bool Intersects(const Plane& other, Ray& inter) const
	{
		auto d = Vector3::Cross(m_Normal, other.m_Normal);

		auto denom = Vector3::Dot(d, d);

		if (denom > 0.0f)
		{
			auto p = Vector3::Cross(m_D * other.m_Normal - other.m_D * m_Normal, d) / denom;

			inter = Ray(p, d.Normalized());
			return true;
		}
		return false;
	}

	bool Intersects(const Plane& otherA, const Plane& otherB, Vector3& inter) const
	{
		auto u = Vector3::Cross(otherA.m_Normal, otherB.m_Normal);
		auto denom = Vector3::Dot(m_Normal, u);

		if (MathU::Abs(denom) > 0.0f)
		{
			inter = (m_D * u + Vector3::Cross(m_Normal, otherB.m_D * otherA.m_Normal - otherA.m_D * otherB.m_Normal)) / denom;
			return true;
		}
		return false;
	}

	const auto& GetNormal() const
	{
		return m_Normal;
	}

	auto GetP0() const
	{
		return m_D * m_Normal;
	}

private:
	Vector3 m_Normal;
	float m_D;
};
