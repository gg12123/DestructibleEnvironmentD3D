#pragma once

template<class T>
class ReadOnlyView
{
public:
	ReadOnlyView(T& coll)
	{
		m_Coll = coll;
	}

	typename T::iterator Begin() const
	{
		return m_Coll.begin();
	}

	typename T::iterator End() const
	{
		return m_Coll.end();
	}

private:
	T * m_Coll;
};
