#pragma once
#include <vector>
#include "ObjectInHGrid.h"

class GridSquaresBucket
{
public:
	const auto& GetObjects() const
	{
		return m_Objects;
	}

	void AddObject(ObjectInHGrid& obj)
	{
		m_Objects.emplace_back(&obj);
	}

	void Clear()
	{
		m_Objects.clear();
	}

private:
	std::vector<ObjectInHGrid*> m_Objects;
};

class HGridLevel
{
private:
	int CalculateHash(int squareXIndex, int squareYIndex, int squareZIndex) const
	{

	}

	bool BucketIsInUse(int bucketIndex) const
	{

	}

	void SetBucketInUse(int bucketIndex)
	{

	}

	GridSquaresBucket& GetBucketLocal(int bucketIndex)
	{
		auto& b = m_Buckets[bucketIndex];

		if (!BucketIsInUse(bucketIndex))
		{
			b.Clear();
			SetBucketInUse(bucketIndex);
		}

		return b;
	}

public:
	struct SquareRange
	{
		int XStart;
		int YStart;
		int ZStart;
		int XEnd;
		int YEnd;
		int ZEnd;
	};

	HGridLevel(int numBuckets, float squareSize)
	{
		m_Buckets.resize(numBuckets);
		m_SquareSize = squareSize;
	}

	SquareRange GetRange(const ObjectInHGrid& obj) const
	{

	}

	int GetBucketIndex(int squareXIndex, int squareYIndex, int squareZIndex) const
	{
		CalculateHash(squareXIndex, squareYIndex, squareZIndex);
	}

	const GridSquaresBucket& GetBucket(int bucketIndex)
	{
		return GetBucketLocal(bucketIndex);
	}

	void Clear()
	{
		// set a flag to indicate that all buckets are not in-use
	}

	void Insert(ObjectInHGrid& toInsert)
	{
		auto range = GetRange(toInsert);

		for (auto x = range.XStart; x != range.XEnd; x++)
		{
			for (auto y = range.YStart; y != range.YEnd; y++)
			{
				for (auto z = range.ZStart; z != range.ZEnd; z++)
					GetBucketLocal(GetBucketIndex(x, y, z)).AddObject(toInsert);
			}
		}
	}

private:
	std::vector<GridSquaresBucket> m_Buckets;
	float m_SquareSize;
};