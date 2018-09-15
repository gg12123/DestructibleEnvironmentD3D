#pragma once
#include <functional>
#include <vector>

template<class T>
class PoolOfRecyclables
{
public:
	PoolOfRecyclables(int initialSize, std::function<T()> creator) : m_Creator(std::move(creator))
	{
		for (auto i = 0; i < initialSize; i++)
			m_Contents.emplace_back(m_Creator());
	}

	void Reset()
	{
		m_Curr = 0;
		m_CurrEnd = Begin();
	}

	T & Recycle()
	{
		if (m_Curr == m_Contents.size())
			m_Contents.emplace_back(m_Creator());

		m_Curr++;
		
		if (m_Curr < m_Contents.size())
		{
			m_CurrEnd = Begin();
			std::advance(m_CurrEnd, m_Curr);
		}
		else
		{
			m_CurrEnd = m_Contents.end();
		}

		return m_Contents[m_Curr - 1];
	}

	auto NumRecycled() const
	{
		return m_Curr;
	}

	auto Begin()
	{
		return m_Contents.begin();
	}

	auto End()
	{
		return m_CurrEnd;
	}

	auto& At(uint16 index)
	{
		assert(index < m_Curr);
		return m_Contents[index];
	}

private:
	std::function<T()> m_Creator;
	std::vector<T> m_Contents;
	uint16 m_Curr;
	typename std::vector<T>::iterator m_CurrEnd;
};
