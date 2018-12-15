#pragma once
#include <array>

template<int SquareSize, class T>
class TriangleArray
{
public:

	// Dont think this is correct
	static constexpr int Capacity = (SquareSize * SquareSize) / 2;

	T & Get(int row, int col)
	{

	}

private:
	std::array<T, Capacity> m_Data;
};
