#pragma once
#include "DynamicTriangleArray.h"
#include "Shape.h"

struct StdVectorRange
{
	int Start;
	int End;

	int Size() const
	{
		return End - Start;
	}

	StdVectorRange(int s, int e)
	{
		Start = s;
		End = e;
	}

	StdVectorRange() : Start(0), End(0)
	{
	}
};

class ContactContext
{
public:
	bool TestedOnPrevTick;
	uint64 TimeStamp = 0ull;

	StdVectorRange ContactPointsRange;
	int ManifoldHistoryIndex;
	int IndexOfSimplex;

	bool InContactOnPrevTick() const
	{
		return TestedOnPrevTick ? m_InContactOnPrevTick : false;
	}

	void SetInContact(bool val)
	{
		m_InContactOnPrevTick = val;
	}

private:
	bool m_InContactOnPrevTick;
};

class ContactContexts
{
public:
	void OnContactFindingStart()
	{
		m_CurrTimeStamp++;

		// TODO - As uint64 is so big I dont think this will ever throw.
		// Must check though.
		assert(m_CurrTimeStamp < (std::numeric_limits<uint64>::max)());
	}

	ContactContext& InitContext(const Shape& shapeA, const Shape& shapeB)
	{
		auto& c = m_Contexts.Get(shapeA.GetShapeId(), shapeB.GetShapeId());
		c.TestedOnPrevTick = (m_CurrTimeStamp == c.TimeStamp);
		c.TimeStamp = m_CurrTimeStamp + 1;
		return c;
	}

	ContactContext& GetContext(const Shape& shapeA, const Shape& shapeB)
	{
		return m_Contexts.Get(shapeA.GetShapeId(), shapeB.GetShapeId());
	}

private:
	DynamicTriangleArray<ContactContext> m_Contexts;
	uint64 m_CurrTimeStamp = 0ull;
};