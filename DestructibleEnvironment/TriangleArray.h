#pragma once
#include <array>

template<int SquareSize, class T>
class TriangleArray
{
private:
	int GetIndex(int row, int col)
	{
		assert(row != col);

		if (col > row)
		{
			auto temp = row;
			row = col;
			col = temp;
		}

		return (row * (row + 1)) / 2 + col;
	}

public:
	static constexpr int Capacity = (SquareSize * (SquareSize - 1)) / 2;

	T & Get(int row, int col)
	{
		return m_Data[GetIndex(row, col)];
	}

private:
	std::array<T, Capacity> m_Data;
};
