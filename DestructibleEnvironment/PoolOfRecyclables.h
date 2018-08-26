#pragma once
#include <functional>
#include <vector>

template<class T>
class PoolOfRecyclables
{
public:
	PoolOfRecyclables(int initialCount, std::function<T()> creator) : m_Creator(std::move(creator))
	{
		for (auto i = 0; i < initialCount; i++)
			m_Contents.emplace_back(m_Creator());
	}

	void Reset()
	{
		m_Curr = 0;
	}

	T & Recycle()
	{
		if (m_Curr == m_Contents.size())
			m_Contents.emplace_back(m_Creator());

		m_Curr++;
		return m_Contents[m_Curr - 1];
	}

private:
	std::function<T()> m_Creator;
	std::vector<T> m_Contents;
	int m_Curr;
};
