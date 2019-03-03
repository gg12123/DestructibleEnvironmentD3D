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

	HGridLevel(int numBuckets, float squareSize) : m_SquareSize(squareSize)
	{
		m_Buckets.resize(numBuckets);
	}

	SquareRange GetRange(const ObjectInHGrid& obj) const
	{
		SquareRange r;

		auto c = obj.GetCentre();
		auto e = obj.GetExtends();

		auto max = c + e;
		auto min = c - e;

		r.XStart = std::floorf(min.x / m_SquareSize);
		r.YStart = std::floorf(min.y / m_SquareSize);
		r.ZStart = std::floorf(min.z / m_SquareSize);

		r.XEnd = std::floorf(max.x / m_SquareSize) + 1;
		r.YEnd = std::floorf(max.y / m_SquareSize) + 1;
		r.ZEnd = std::floorf(max.z / m_SquareSize) + 1;

		return r;
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