#pragma once
#include <vector>
#include "MathU.h"

template<class T>
class DynamicArray
{
private:
	void ReSize(size_t requiredSize)
	{
		if (requiredSize > m_Data.size())
			m_Data.resize(requiredSize);
	}

public:
	DynamicArray<T>& operator=(DynamicArray<T>&& other) = default;
	DynamicArray(DynamicArray<T>&& other) = default;
	DynamicArray() = default;

	T& operator[](size_t index)
	{
		static_assert(std::is_nothrow_move_constructible<DynamicArray<T>>::value, "Dynamic array should be noexcept MoveConstructible");
		ReSize(index + 1u);
		return m_Data[index];
	}

	const T& operator[](size_t index) const
	{
		return m_Data[index];
	}

	void Zero()
	{
		std::memset(m_Data.data(), 0, m_Data.size() * sizeof(T));
	}

private:
	std::vector<T> m_Data;
};