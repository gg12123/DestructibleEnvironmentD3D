#pragma once
#include <stack>
#include <functional>

template<class T>
class Pool
{
public:
	Pool(std::function<T()> creator, int initalSize) : m_Creator(std::move(creator))
	{
		for (int i = 0; i < initalSize; i++)
			m_Objects.push(m_Creator());
	}

	T GetObject()
	{
		if (m_Objects.size() > 0)
		{
			auto obj = std::move(m_Objects.top());
			m_Objects.pop();
			return obj;
		}
		return m_Creator();
	}

	template<class Tin>
	void Return(Tin&& toRet)
	{
		m_Objects.push(std::forward<Tin>(toRet));
	}

private:
	std::stack<T> m_Objects;
	std::function<T()> m_Creator;
};
