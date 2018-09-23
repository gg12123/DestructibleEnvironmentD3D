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
		return "X = " + std::to_string(val.x) + " Y = " + std::to_string(val.y) + " Z = " + std::to_string(val.z);
	}

	template<>
	static std::string ToString<std::string>(const std::string& val)
	{
		return val;
	}
};