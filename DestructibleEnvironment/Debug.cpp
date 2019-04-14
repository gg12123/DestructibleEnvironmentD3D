#include "pch.h"
#include "Debug.h"
#include <Windows.h>

void Debug::LogStr(const std::string& str)
{
	OutputDebugStringA(str.c_str());
}