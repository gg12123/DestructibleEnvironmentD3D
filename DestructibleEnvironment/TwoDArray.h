#pragma once
#include <array>

template<int NumRows, int NumCols, class T>
class TwoDArray
{
public:
	static constexpr int Size = NumRows * NumCols;

	void Clear(const T val)
	{
		for (int i = 0; i < Size; i++)
			m_Data[i] = val;
	}

	T& Get(int row, int col)
	{
		return m_Data[Index(row, col)];
	}

private:
	int Index(int row, int col)
	{
		return (row * NumCols + col);
	}

	std::array<T, Size> m_Data;
};
