#pragma once
#include <limits>

template<class T>
class UpdatableBound
{
public:
	UpdatableBound()
	{
		m_Min = std::numeric_limits<T>::max();
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

private:
	T m_Min;
	T m_Max;
};