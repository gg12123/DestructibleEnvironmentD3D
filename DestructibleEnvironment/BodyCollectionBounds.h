#pragma once
#include "Bounds.h"

class BodyCollectionBounds
{
public:
	template<class Tcollec, class TBounds>
	void Calculate(const Tcollec& bodies)
	{
		m_XAverageRange = 0.0f;
		m_YAverageRange = 0.0f;
		m_ZAverageRange = 0.0f;

		m_XMax = MathU::NegativeInfinity;
		m_YMax = MathU::NegativeInfinity;
		m_ZMax = MathU::NegativeInfinity;

		m_XMin = MathU::NegativeInfinity;
		m_YMin = MathU::NegativeInfinity;
		m_ZMin = MathU::NegativeInfinity;

		for (auto it = bodies.begin(); it != bodies.end(); it++)
		{
			auto& body = **it;
			auto& bounds = body.GetWorldBounds();

			body.UpdateWorldBounds(BoundsType<TBounds>::Value);

			if (bounds.GetXMin() < m_XMin)
				m_XMin = bounds.GetXMin();

			if (bounds.GetYMin() < m_YMin)
				m_YMin = bounds.GetYMin();

			if (bounds.GetZMin() < m_ZMin)
				m_ZMin = bounds.GetZMin();

			if (bounds.GetXMax() > m_XMax)
				m_XMax = bounds.GetXMax();

			if (bounds.GetYMax() > m_YMax)
				m_YMax = bounds.GetYMax();

			if (bounds.GetZMax() > m_ZMax)
				m_ZMax = bounds.GetZMax();

			m_XAverageRange += bounds.GetXRange();
			m_YAverageRange += bounds.GetYRange();
			m_ZAverageRange += bounds.GetZRange();
		}

		auto c = static_cast<float>(bodies.size());
		m_XAverageRange /= c;
		m_YAverageRange /= c;
		m_ZAverageRange /= c;
	}

	float GetCollectionXRange() const
	{
		return m_XMax - m_XMin;
	}

	float GetCollectionYRange() const
	{
		return m_YMax - m_YMin;
	}

	float GetCollectionZRange() const
	{
		return m_ZMax - m_ZMin;
	}

	float GetXMin() const
	{
		return m_XMin;
	}

	float GetYMin() const
	{
		return m_YMin;
	}

	float GetZMin() const
	{
		return m_ZMin;
	}

	float GetXAverageRange() const
	{
		return m_XAverageRange;
	}

	float GetYAverageRange() const
	{
		return m_YAverageRange;
	}

	float GetZAverageRange() const
	{
		return m_ZAverageRange;
	}

private:

	float m_XMin;
	float m_YMin;
	float m_ZMin;

	float m_XMax;
	float m_YMax;
	float m_ZMax;

	float m_XAverageRange;
	float m_YAverageRange;
	float m_ZAverageRange;

	// could also have min and max range
};