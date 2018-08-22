#pragma once

template<class T, int N>
class ArrayWrapper
{
public:
	ArrayWrapper()
	{
		m_CurrCount = 0;
	}

	template<class Tin>
	void Add(Tin&& toAdd)
	{
		assert(m_CurrCount < N);
		m_Data[m_CurrCount] = std::forward<Tin>(toAdd);
		m_CurrCount++;
	}

	int GetCurrCount()
	{
		return m_CurrCount;
	}

	T* GetData()
	{
		return m_Data;
	}

	void Clear()
	{
		m_CurrCount = 0;
	}

private:
	int m_CurrCount;
	T m_Data[N];
};