#pragma once

template<class T, int N>
class ArrayWrapper
{
public:
	ArrayWrapper()
	{
		m_CurrCount = 0;
	}

	void Add(T&& toAdd)
	{
		assert(m_CurrCount < N);
		m_Data[m_CurrCount] = std::forward<T>(toAdd);
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

private:
	int m_CurrCount;
	T m_Data[N];
};