#pragma once

template<class T>
class LastCheckedAgainst
{
public:
	void SetLastCheckedAgainst(T last)
	{
		m_Last = last;
	}

	T GetLastCheckedAgainst()
	{
		return m_Last;
	}

private:
	T m_Last;
};
