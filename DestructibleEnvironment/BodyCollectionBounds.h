#pragma once
#include "Bounds.h"

class BodyCollectionBounds
{
public:
	template<class Tcollec, class TBounds>
	void Calculate(const Tcollec& bodies)
	{

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