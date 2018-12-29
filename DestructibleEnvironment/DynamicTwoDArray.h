#pragma once
#include "DynamicArray.h"

template<class T>
class DynamicTwoDArray
{
public:
	T & Get(int row, int col)
	{
		return m_Data[row][col];
	}

	const T & Get(int row, int col) const
	{
		return m_Data[row][col];
	}

private:
	DynamicArray<DynamicArray<T>> m_Data;
};
