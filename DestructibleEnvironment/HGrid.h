#pragma once
#include <vector>
#include <stack>
#include <algorithm>
#include "HGridLevel.h"

class HGrid
{
private:
	class LevelWithObjects
	{
	private:
		void Query(const ObjectInHGrid& obj, int bucketIndex)
		{
			auto& bucket = m_Level.GetBucket(bucketIndex);

			for (auto& other : bucket.GetObjects())
			{
				// check the other object has not already been checked against
				// the input obj using the 'last checked against' flag.
				// then check for collision
			}
		}

	public:
		LevelWithObjects(int numBuckets, float squareSize) : m_Level(numBuckets, squareSize)
		{
		}

		void AddObject(ObjectInHGrid& obj)
		{
			m_Objects.push(&obj);
		}

		void Query(const ObjectInHGrid& obj) // also pass in an object to call back to for handling the collision test
		{
			auto range = m_Level.GetRange(obj);

			for (auto x = range.XStart; x != range.XEnd; x++)
			{
				for (auto y = range.YStart; y != range.YEnd; y++)
				{
					for (auto z = range.ZStart; z != range.ZEnd; z++)
						Query(obj, m_Level.GetBucketIndex(x, y, z));
				}
			}
		}

		ObjectInHGrid* GetNextToInsert() const
		{
			return m_Objects.size() > 0u ? m_Objects.top() : nullptr;
		}

		void InsertNext()
		{
			auto& toInsert = *GetNextToInsert();
			m_Objects.pop();

			// Clear 'last checked against' on the object to insert

			m_Level.Insert(toInsert);
		}

		void ClearLevel()
		{
			m_Level.Clear();
		}

	private:
		HGridLevel m_Level;
		std::stack<ObjectInHGrid*> m_Objects;
	};

	float CalculateObjectSize(const ObjectInHGrid& obj) const
	{
		auto ex = obj.GetExtends();
		auto maxEx = MathU::Max(MathU::Max(ex.x, ex.y), ex.z);
		return 2.0f * maxEx;
	}

	int GetLevel(const ObjectInHGrid& obj) const
	{
		auto x = static_cast<int>(std::floorf((CalculateObjectSize(obj) - m_MinObjSize) / m_MinObjSize));
		return m_LevelLookup[MathU::Clamp(x, 0, static_cast<int>(m_Levels.size()) - 1)];
	}

	int NumBucketsFromSquareSize(float size) const
	{
		return 1000; // TODO - use less buckets when the square size is bigger
	}

public:
	HGrid(float maxObjSize, float minObjSize)
	{
		auto size = maxObjSize;
		m_Levels.emplace_back(LevelWithObjects(NumBucketsFromSquareSize(size), size));

		while (size > minObjSize)
		{
			size /= 2.0f;
			m_Levels.emplace_back(LevelWithObjects(NumBucketsFromSquareSize(size), size));
		}

		m_MinObjSize = size;

		auto insertCount = 1;
		auto insertVal = m_Levels.size() - 1;
		for (auto i = 0u; i < m_Levels.size(); i++)
		{
			for (auto i = 0; i < insertCount; i++)
				m_LevelLookup.emplace_back(insertVal);

			insertCount *= 2;
			insertVal--;
		}
	}

	void AddObject(ObjectInHGrid& obj)
	{
		m_Levels[GetLevel(obj)].AddObject(obj);
	}

	void HandleLevel(int levelIndex)
	{
		auto& level = m_Levels[levelIndex];
		level.ClearLevel();

		auto nextObj = level.GetNextToInsert();
		while (nextObj)
		{
			// Test for collisions against its own level, and all levels
			// that contain larger objects
			for (auto i = levelIndex; i >= 0; i--)
				m_Levels[i].Query(*nextObj);

			// Now insert the object into its level.
			level.InsertNext();

			nextObj = level.GetNextToInsert();
		}
	}

	void Run()
	{
		for (auto i = 0u; i < m_Levels.size(); i++)
			HandleLevel(i);
	}

private:
	std::vector<LevelWithObjects> m_Levels; // index 0 is for largest objects
	std::vector<int> m_LevelLookup;
	float m_MinObjSize;
};