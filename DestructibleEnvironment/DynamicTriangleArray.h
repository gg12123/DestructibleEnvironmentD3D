#pragma once
#include "DynamicArray.h"

template<class T>
class DynamicTriangleArray
{
private:
	int GetIndex(int row, int col) const
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
	T & Get(int row, int col)
	{
		return m_Data[GetIndex(row, col)];
	}

	const T & Get(int row, int col) const
	{
		return m_Data[GetIndex(row, col)];
	}

	void Zero()
	{
		m_Data.Zero();
	}

private:
	DynamicArray<T> m_Data;
};
