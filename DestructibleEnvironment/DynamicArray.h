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
		{
			auto oldSize = m_Data.size();
			m_Data.reserve(requiredSize);

			for (auto i = oldSize; i < requiredSize; i++)
				m_Data.emplace_back(T());
		}
	}

public:
	DynamicArray<T>& operator=(DynamicArray<T>&& other) = default;
	DynamicArray(DynamicArray<T>&& other) = default;

	DynamicArray()
	{
		static_assert(std::is_nothrow_move_constructible<DynamicArray<T>>::value, "Dynamic array should be noexcept MoveConstructible");
	}

	T& operator[](size_t index)
	{
		ReSize(index + 1u);
		return m_Data[index];
	}

	const T& operator[](size_t index) const
	{
		return m_Data[index];
	}

	void Clear(int val)
	{
		std::memset(m_Data.data(), val, m_Data.size() * sizeof(T));
	}

private:
	std::vector<T> m_Data;
};
