#pragma once
#include <vector>
#include <stack>
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

	int GetLevel(const ObjectInHGrid& obj) const
	{

	}

public:
	HGrid(float maxObjSize, float minObjSize)
	{

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
};