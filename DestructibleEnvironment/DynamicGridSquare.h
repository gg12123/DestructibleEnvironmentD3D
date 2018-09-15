#pragma once
#include <vector>

class Rigidbody;

class DynamicGridSquare
{
public:
	DynamicGridSquare(uint16 index)
	{
		m_Index = index;
	}

	void AddAndDetect(Rigidbody& toAdd);

	bool WasPlacedOnGridThisTick(uint16 numSqauresAdded)
	{
		return (m_Index < numSqauresAdded);
	}

	void Reset()
	{
		m_Contents.clear();
	}

private:
	uint16 m_Index;
	std::vector<Rigidbody*> m_Contents; // TODO - use something that can be cleared in constant time.
};
