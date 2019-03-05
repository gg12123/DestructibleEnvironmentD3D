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
		if (std::abs(m_Centre.x - other.m_Centre.x) > (m_Extends.x + other.m_Extends.x))
			return false;

		if (std::abs(m_Centre.y - other.m_Centre.y) > (m_Extends.y + other.m_Extends.y))
			return false;

		if (std::abs(m_Centre.z - other.m_Centre.z) > (m_Extends.z + other.m_Extends.z))
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
		if (newPoint.x > m_XMax)
			m_XMax = newPoint.x;

		if (newPoint.x < m_XMin)
			m_XMin = newPoint.x;

		if (newPoint.y > m_YMax)
			m_YMax = newPoint.y;

		if (newPoint.y < m_YMin)
			m_YMin = newPoint.y;

		if (newPoint.z > m_ZMax)
			m_ZMax = newPoint.z;

		if (newPoint.z < m_ZMin)
			m_ZMin = newPoint.z;
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