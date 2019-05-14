#pragma once
#include "Vector3.h"
#include "MathU.h"

class AABB
{
public:
	AABB()
	{
	}

	AABB(const Vector3& centre, const Vector3& extends) : m_Centre(centre), m_Extends(extends)
	{
	}

	Vector3 GetCentre() const
	{
		return m_Centre;
	}

	Vector3 GetExtends() const
	{
		return m_Extends;
	}

	Vector3 GetSize() const
	{
		return 2.0f * m_Extends;
	}

	bool OverlapsWith(const AABB& other) const
	{
		if (std::abs(m_Centre.X() - other.m_Centre.X()) > (m_Extends.X() + other.m_Extends.X()))
			return false;

		if (std::abs(m_Centre.Y() - other.m_Centre.Y()) > (m_Extends.Y() + other.m_Extends.Y()))
			return false;

		if (std::abs(m_Centre.Z() - other.m_Centre.Z()) > (m_Extends.Z() + other.m_Extends.Z()))
			return false;

		return true;
	}

	void Fatten(float amount)
	{
		m_Extends.Floats[0] += amount;
		m_Extends.Floats[1] += amount;
		m_Extends.Floats[2] += amount;
	}

	bool IntersectsSegment(const Vector3& p0, const Vector3& p1) const
	{
		auto& c = m_Centre;
		auto& e = m_Extends;
		auto m = (p0 + p1) / 2.0f;
		auto d = p1 - m;
		m -= c;

		auto adx = MathU::Abs(d.X());
		if (MathU::Abs(m.X()) > e.X() + adx)
			return false;

		auto ady = MathU::Abs(d.Y());
		if (MathU::Abs(m.Y()) > e.Y() + ady)
			return false;

		auto adz = MathU::Abs(d.Z());
		if (MathU::Abs(m.Z()) > e.Z() + adz)
			return false;

		adx += MathU::Epsilon;
		ady += MathU::Epsilon;
		adz += MathU::Epsilon;

		if (MathU::Abs(m.Y() * d.Z() - m.Z() * d.Y()) > e.Y() * adz + e.Z() * ady)
			return false;

		if (MathU::Abs(m.Z() * d.X() - m.X() * d.Z()) > e.X() * adz + e.Z() * adx)
			return false;

		if (MathU::Abs(m.X() * d.Y() - m.Y() * d.X()) > e.X() * ady + e.Y() * adx)
			return false;

		return true;
	}

	bool ContainsPoint(const Vector3& p) const
	{
		auto min = m_Centre - m_Extends;
		if (p.X() < min.X())
			return false;
		if (p.Y() < min.Y())
			return false;
		if (p.Z() < min.Z())
			return false;

		auto max = m_Centre + m_Extends;
		if (p.X() > max.X())
			return false;
		if (p.Y() > max.Y())
			return false;
		if (p.Z() > max.Z())
			return false;

		return true;
	}

private:
	Vector3 m_Centre;
	Vector3 m_Extends;
};

class BoundsCalculator
{
public:
	BoundsCalculator()
	{
		Reset();
	}

	void Reset()
	{
		m_XMin = MathU::Infinity;
		m_XMax = MathU::NegativeInfinity;

		m_YMin = MathU::Infinity;
		m_YMax = MathU::NegativeInfinity;

		m_ZMin = MathU::Infinity;
		m_ZMax = MathU::NegativeInfinity;
	}

	void Update(const Vector3& newPoint)
	{
		if (newPoint.X() > m_XMax)
			m_XMax = newPoint.X();

		if (newPoint.X() < m_XMin)
			m_XMin = newPoint.X();

		if (newPoint.Y() > m_YMax)
			m_YMax = newPoint.Y();

		if (newPoint.Y() < m_YMin)
			m_YMin = newPoint.Y();

		if (newPoint.Z() > m_ZMax)
			m_ZMax = newPoint.Z();

		if (newPoint.Z() < m_ZMin)
			m_ZMin = newPoint.Z();
	}

	float GetXMin() const
	{
		return m_XMin;
	}

	float GetXMax() const
	{
		return m_XMax;
	}

	float GetYMin() const
	{
		return m_YMin;
	}

	float GetYMax() const
	{
		return m_YMax;
	}

	float GetZMin() const
	{
		return m_ZMin;
	}

	float GetZMax() const
	{
		return m_ZMax;
	}

	float GetXRange() const
	{
		return m_XMax - m_XMin;
	}

	float GetYRange() const
	{
		return m_YMax - m_YMin;
	}

	float GetZRange() const
	{
		return m_ZMax - m_ZMin;
	}

	AABB ToAABB() const
	{
		return AABB(Vector3((m_XMax + m_XMin) / 2.0f, (m_YMax + m_YMin) / 2.0f, (m_ZMax + m_ZMin) / 2.0f),
			Vector3(0.5f * GetXRange(), 0.5f * GetYRange(), 0.5f * GetZRange()));
	}

private:
	float m_XMin;
	float m_XMax;

	float m_YMin;
	float m_YMax;

	float m_ZMin;
	float m_ZMax;
};