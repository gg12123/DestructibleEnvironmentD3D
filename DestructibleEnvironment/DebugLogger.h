#pragma once
#include <iostream>
#include <fstream>
#include "StringU.h"

// dont make it a singleton becasue i will need different instances for different threads
class DebugLogger
{
public:
	~DebugLogger()
	{
		CloseStream();
	}

	template<class Tstr>
	void SetPath(Tstr&& fileName)
	{
		CloseStream();
		m_Stream.open(std::forward<Tstr>(fileName));
	}

	template<class Tstr>
	bool TrySetPath(Tstr&& fileName)
	{
		if (!m_Stream.is_open())
		{
			m_Stream.open(std::forward<Tstr>(fileName));
			assert(m_Stream.is_open());
			return true;
		}
		return false;
	}

	template<class T>
	void Log(const T& val)
	{
		WriteToFile(StringU::ToString(val));
	}

private:
	void WriteToFile(const std::string& str)
	{
		m_Stream << str + "\n";
	}

	void CloseStream()
	{
		if (m_Stream.is_open())
			m_Stream.close();
	}

	std::ofstream m_Stream;
};
