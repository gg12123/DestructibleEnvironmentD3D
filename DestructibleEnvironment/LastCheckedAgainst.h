#pragma once

template<class T>
class LastCheckedAgainst
{
public:
	T GetLastCheckedAgainst() const
	{
		return m_Val;
	}

	void SetLastCheckedAgainst(const T& val)
	{
		m_Val = val;
	}

private:
	T m_Val;
};
