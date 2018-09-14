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

private:
	float m_XMin;
	float m_XMax;

	float m_YMin;
	float m_YMax;

	float m_ZMin;
	float m_ZMax;
};