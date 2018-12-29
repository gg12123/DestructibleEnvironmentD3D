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

	void Clear(int val)
	{
		m_Data.Clear(val);
	}

private:
	DynamicArray<T> m_Data;
};
