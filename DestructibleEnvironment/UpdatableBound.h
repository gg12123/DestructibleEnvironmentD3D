#pragma once
#include <limits>
#include "MathU.h"

template<class T>
class UpdatableBound
{
public:
	UpdatableBound()
	{
		m_Min = (std::numeric_limits<T>::max)();
		m_Max = std::numeric_limits<T>::lowest();
	}

	void Update(T val)
	{
		if (val < m_Min)
			m_Min = val;

		if (val > m_Max)
			m_Max = val;
	}

	const T& GetMin() const
	{
		return m_Min;
	}

	const T& GetMax() const
	{
		return m_Max;
	}

	bool Overlaps(const UpdatableBound<T>& other) const
	{
		return (m_Max > other.m_Min) && (other.m_Max > m_Min);
	}

private:
	T m_Min;
	T m_Max;
};