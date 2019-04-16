#pragma once
#include "StringU.h"

class Debug
{
private:
	static void LogStr(const std::string& str);

public:
	template<class T>
	static void Log(const T& val)
	{
		LogStr(StringU::ToString(val) + "\n");
	}
};