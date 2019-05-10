#pragma once
#include <vector>

template<class T, class Talloc = allocator<T>>
class UnOrderedVector
{
public:
	class Pointer
	{
	public:
		virtual int GetIndex() const = 0;
	};

private:
	class PointerInternal : public Pointer
	{
	public:
		int Index;
		int GetIndex() const override
		{
			return Index;
		}

		PointerInternal(int index)
		{
			Index = index;
		}
	};

public:
	UnOrderedVector() : m_Size(0)
	{
	}

	~UnOrderedVector()
	{
		for (auto p : m_Pointers)
			delete p;
	}

	Pointer * Add(const T& val)
	{
		PointerInternal* p;

		if (m_Size < m_Values.size())
		{
			m_Values[m_Size] = val;
			p = m_Pointers[m_Size];
			p->Index = m_Size;
		}
		else
		{
			m_Values.emplace_back(val);
			p = new PointerInternal(m_Size);
			m_Pointers.emplace_back(p);
		}

		m_Size++;
		return p;
	}

	void Remove(Pointer* pointer)
	{
		auto toRemove = pointer->GetIndex();
		auto end = m_Size - 1;
		auto internalPointer = m_Pointers[toRemove];
		
		// Move end to the free slot
		m_Values[toRemove] = m_Values[end];
		m_Pointers[toRemove] = m_Pointers[end];
		m_Pointers[toRemove]->Index = toRemove;

		// Move the old pointer to the end
		m_Pointers[end] = internalPointer;
		m_Pointers[end]->Index = end;

		m_Size--;
	}

	// Only key into the collection. Do not iteratate over it.
	auto& GetValues()
	{
		return m_Values;
	}

	auto Begin()
	{
		return m_Values.begin();
	}

	auto End()
	{
		return m_Values.begin() + m_Size;
	}

	auto& GetValue(Pointer* pointer)
	{
		return m_Values[pointer->GetIndex()];
	}

private:
	std::vector<T, Talloc> m_Values;
	std::vector<PointerInternal*> m_Pointers;

	uint32 m_Size;
};
