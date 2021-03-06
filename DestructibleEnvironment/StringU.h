#pragma once
#include <string>
#include "Vector3.h"

class StringU
{
public:

	template<class T>
	static std::string ToString(const T& val)
	{
		return std::to_string(val);
	}

	template<>
	static std::string ToString<Vector3>(const Vector3& val)
	{
		return "X = " + std::to_string(val.X()) + " Y = " + std::to_string(val.Y()) + " Z = " + std::to_string(val.Z());
	}

	template<>
	static std::string ToString<std::string>(const std::string& val)
	{
		return val;
	}

	//template<>
	//static std::string ToString<char*>(const char*& val)
	//{
	//	return std::string(val);
	//}
};