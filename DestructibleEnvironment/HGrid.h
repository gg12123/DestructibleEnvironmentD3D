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
		template<CollisionObjectType objType, CollisionObjectType othersType>
		void Query(Tobject& obj, const GridSquaresBucket<Tobject>& bucket, TcollisionHandler& collHandler)
		{
			for (auto other : bucket.GetObjects<othersType>())
			{
				// check the other object has not already been checked against
				// the input obj using the 'last checked against' flag.
				// then check for collision
				if (other->GetLastCheckedAgainst() != &obj)
				{
					other->SetLastCheckedAgainst(&obj);

					if (other->GetWorldAABB().OverlapsWith(obj.GetWorldAABB()))
						collHandler.RunNarrowPhaseCheckForCollision<objType, othersType>(obj, *other);
				}
			}
		}

		template<CollisionObjectType type>
		auto& GetObjects()
		{
			return m_ObjectsRealPhysical;
		}

		template<>
		auto& GetObjects<CollisionObjectType::Trigger>()
		{
			return m_ObjectsTrigger;
		}

		template<>
		auto& GetObjects<CollisionObjectType::CharController>()
		{
			return m_ObjectsCharController;
		}

	public:
		LevelWithObjects(int numBuckets, float squareSize) : m_Level(numBuckets, squareSize)
		{
		}

		template<CollisionObjectType type>
		void AddObject(Tobject& obj)
		{
			GetObjects<type>().push(&obj);
		}

		template<CollisionObjectType objType>
		void Query(Tobject& obj, TcollisionHandler& collHandler)
		{
			auto range = m_Level.GetRange(obj);

			for (auto x = range.XStart; x != range.XEnd; x++)
			{
				for (auto y = range.YStart; y != range.YEnd; y++)
				{
					for (auto z = range.ZStart; z != range.ZEnd; z++)
					{
						auto& bucket = m_Level.GetBucket(m_Level.GetBucketIndex(x, y, z));

						Query<objType, CollisionObjectType::RealPhysical>(obj, bucket, collHandler);
						Query<objType, CollisionObjectType::Trigger>(obj, bucket, collHandler);
						Query<objType, CollisionObjectType::CharController>(obj, bucket, collHandler);
					}
				}
			}
		}

		template<CollisionObjectType type>
		Tobject* GetNextToInsert() const
		{
			auto& objs = GetObjects<type>();
			return objs.size() > 0u ? objs.top() : nullptr;
		}

		template<CollisionObjectType type>
		void InsertNext()
		{
			auto& toInsert = *GetNextToInsert<type>();
			m_Objects.pop();

			toInsert.SetLastCheckedAgainst(nullptr);
			m_Level.Insert<type>(toInsert);
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

		std::stack<Tobject*> m_ObjectsRealPhysical;
		std::stack<Tobject*> m_ObjectsTrigger;
		std::stack<Tobject*> m_ObjectsCharController;
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

	template<CollisionObjectType type>
	void HandleActiveLevel(int levelIndex, TcollisionHandler& collHandler)
	{
		auto& level = *m_ActiveLevels[levelIndex];
		level.ClearLevel();

		auto nextObj = level.GetNextToInsert<type>();
		while (nextObj)
		{
			// Test for collisions against its own level, and all levels
			// that contain larger objects
			for (auto i = levelIndex; i >= 0; i--)
				m_ActiveLevels[i]->Query<type>(*nextObj, collHandler);

			// Now insert the object into its level.
			level.InsertNext<type>();

			nextObj = level.GetNextToInsert<type>();
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

	template<CollisionObjectType type>
	void AddObject(Tobject& obj)
	{
		m_Levels[GetLevel(obj)].AddObject<type>(obj);
	}

	void Run(TcollisionHandler& collHandler)
	{
		CollectActiveLevels();

		for (auto i = 0u; i < m_ActiveLevels.size(); i++)
		{
			HandleActiveLevel<CollisionObjectType::RealPhysical>(i, collHandler);
			HandleActiveLevel<CollisionObjectType::Trigger>(i, collHandler);
			HandleActiveLevel<CollisionObjectType::CharController>(i, collHandler);
		}
	}

private:
	std::vector<LevelWithObjects> m_Levels; // index 0 is for largest objects
	std::vector<LevelWithObjects*> m_ActiveLevels;
	std::vector<int> m_LevelLookup;
	float m_MinObjSize;
};