#pragma once

template<class T>
class ReadOnlyView
{
public:
	ReadOnlyView(const std::vector<T>& coll)
	{
		m_Coll = &coll;
	}

	const auto Begin() const
	{
		return m_Coll.begin();
	}

	const auto End() const
	{
		return m_Coll.end();
	}

	const auto Data() const
	{
		return m_Coll.data();
	}

	auto Size() const
	{
		return m_Coll.size();
	}

private:
	const std::vector<T> * m_Coll;
};
