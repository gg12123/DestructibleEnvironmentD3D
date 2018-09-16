#pragma once
#include "Vector3.h"
#include "MathUtils.h"

class Bounds
{
public:
	Bounds()
	{
		Reset();
	}

	void Reset()
	{
		m_XMin = MathUtils::Infinity;
		m_XMax = MathUtils::NegativeInfinity;

		m_YMin = MathUtils::Infinity;
		m_YMax = MathUtils::NegativeInfinity;

		m_ZMin = MathUtils::Infinity;
		m_ZMax = MathUtils::NegativeInfinity;
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

	float GetVolume() const
	{
		return (m_XMax - m_XMin) * (m_YMax - m_YMin) * (m_ZMax - m_ZMin);
	}

	void ConstrcutFromCentreAndRadius(const Vector3& centre, float radius)
	{

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

private:
	float m_XMin;
	float m_XMax;

	float m_YMin;
	float m_YMax;

	float m_ZMin;
	float m_ZMax;
};

struct RadiusBoundsType
{
};

struct AABBBoundsType
{
};

template<class T>
struct BoundsType
{
	static constexpr T Value = T();
};