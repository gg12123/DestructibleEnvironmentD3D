#pragma once
#include <vector>
#include <stack>
#include <algorithm>
#include "HGridLevel.h"

template<class Tobject, class TcollisionHandler>
class HGrid
{
private:
	class LevelWithObjects
	{
	private:
		void Query(Tobject& obj, int bucketIndex, TcollisionHandler& collHandler)
		{
			auto& bucket = m_Level.GetBucket(bucketIndex);

			for (auto other : bucket.GetObjects())
			{
				// check the other object has not already been checked against
				// the input obj using the 'last checked against' flag.
				// then check for collision
				if (other->GetLastCheckedAgainst() != &obj)
				{
					other->SetLastCheckedAgainst(&obj);

					if (other->GetWorldAABB().OverlapsWith(obj.GetWorldAABB()))
						collHandler.RunNarrowPhaseCheckForCollision(obj, *other);
				}
			}
		}

	public:
		LevelWithObjects(int numBuckets, float squareSize) : m_Level(numBuckets, squareSize)
		{
		}

		void AddObject(Tobject& obj)
		{
			m_Objects.push(&obj);
		}

		void Query(Tobject& obj, TcollisionHandler& collHandler) // also pass in an object to call back to for handling the collision test
		{
			auto range = m_Level.GetRange(obj);

			for (auto x = range.XStart; x != range.XEnd; x++)
			{
				for (auto y = range.YStart; y != range.YEnd; y++)
				{
					for (auto z = range.ZStart; z != range.ZEnd; z++)
						Query(obj, m_Level.GetBucketIndex(x, y, z), collHandler);
				}
			}
		}

		Tobject* GetNextToInsert() const
		{
			return m_Objects.size() > 0u ? m_Objects.top() : nullptr;
		}

		void InsertNext()
		{
			auto& toInsert = *GetNextToInsert();
			m_Objects.pop();

			toInsert.SetLastCheckedAgainst(nullptr);
			m_Level.Insert(toInsert);
		}

		void ClearLevel()
		{
			m_Level.Clear();
		}

		bool HasObjectsToInsert() const
		{
			return m_Objects.size() > 0u;
		}

	private:
		HGridLevel<Tobject> m_Level;
		std::stack<Tobject*> m_Objects;
	};

	float CalculateObjectSize(const Tobject& obj) const
	{
		auto ex = obj.GetWorldAABB().GetExtends();
		auto maxEx = MathU::Max(MathU::Max(ex.X(), ex.Y()), ex.Z());
		return 2.0f * maxEx;
	}

	int GetLevel(const Tobject& obj) const
	{
		auto x = static_cast<int>(std::floorf((CalculateObjectSize(obj) - m_MinObjSize) / m_MinObjSize));
		return m_LevelLookup[MathU::Clamp(x, 0, static_cast<int>(m_Levels.size()) - 1)];
	}

	int NumBucketsFromSquareSize(float size) const
	{
		return 1000; // TODO - use less buckets when the square size is bigger
	}

	void CollectActiveLevels()
	{
		m_ActiveLevels.clear();
		for (auto& l : m_Levels)
		{
			if (l.HasObjectsToInsert())
				m_ActiveLevels.emplace_back(&l);
		}
	}

	void HandleActiveLevel(int levelIndex, TcollisionHandler& collHandler)
	{
		auto& level = *m_ActiveLevels[levelIndex];
		level.ClearLevel();

		auto nextObj = level.GetNextToInsert();
		while (nextObj)
		{
			// Test for collisions against its own level, and all levels
			// that contain larger objects
			for (auto i = levelIndex; i >= 0; i--)
				m_ActiveLevels[i]->Query(*nextObj, collHandler);

			// Now insert the object into its level.
			level.InsertNext();

			nextObj = level.GetNextToInsert();
		}
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

	void AddObject(Tobject& obj)
	{
		m_Levels[GetLevel(obj)].AddObject(obj);
	}

	void Run(TcollisionHandler& collHandler)
	{
		CollectActiveLevels();

		for (auto i = 0u; i < m_ActiveLevels.size(); i++)
			HandleActiveLevel(i, collHandler);
	}

private:
	std::vector<LevelWithObjects> m_Levels; // index 0 is for largest objects
	std::vector<LevelWithObjects*> m_ActiveLevels;
	std::vector<int> m_LevelLookup;
	float m_MinObjSize;
};