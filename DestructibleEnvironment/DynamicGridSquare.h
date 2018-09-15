#pragma once
#include <vector>
#include "CollisionData.h"

class Rigidbody;
class CollisionDetector;
class CollisionResponder;

class DynamicGridSquare
{
public:
	DynamicGridSquare(uint32 index, CollisionDetector& detector, CollisionResponder& responder)
	{
		m_Index = index;
		m_Detecter = &detector;
		m_Responder = &responder;
	}

	void AddAndDetect(Rigidbody& toAdd);

	bool WasPlacedOnGridThisTick(uint32 numSqauresAdded)
	{
		return (m_Index < numSqauresAdded);
	}

	void Reset()
	{
		m_Contents.clear();
	}

private:
	uint32 m_Index;

	std::vector<Rigidbody*> m_Contents; // TODO - use something that can be cleared in constant time.

	CollisionDetector* m_Detecter;
	CollisionResponder* m_Responder;

	CollisionData m_CollData;
};
