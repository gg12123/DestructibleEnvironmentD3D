#pragma once
#include <vector>
#include "ObjectInHGrid.h"

template<class Tobject>
class GridSquaresBucket
{
public:
	const auto& GetObjects() const
	{
		return m_Objects;
	}

	void AddObject(Tobject& obj)
	{
		m_Objects.emplace_back(&obj);
	}

	void Clear()
	{
		m_Objects.clear();
	}

private:
	std::vector<Tobject*> m_Objects;
};

template<class Tobject>
class HGridLevel
{
private:
	int CalculateHash(int squareXIndex, int squareYIndex, int squareZIndex) const
	{
		static constexpr int32 h1 = 0x8da6b343;
		static constexpr int32 h2 = 0xd8163841;
		static constexpr int32 h3 = 0xcb1ab31f;

		auto n = (h1 * squareXIndex + h2 * squareYIndex + h3 * squareZIndex) % m_Buckets.size();

		if (n < 0)
			n += m_Buckets.size();

		return n;
	}

	bool BucketIsInUse(int bucketIndex) const
	{
		return m_TimeStamps[bucketIndex] == m_CurrStamp;
	}

	void SetBucketInUse(int bucketIndex)
	{
		m_TimeStamps[bucketIndex] = m_CurrStamp;
	}

	void ClearTimeStamps()
	{
		std::memset(m_TimeStamps.data(), 0, m_TimeStamps.size() * sizeof(uint64));
	}

	GridSquaresBucket<Tobject>& GetBucketLocal(int bucketIndex)
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
		m_TimeStamps.resize(numBuckets);
		ClearTimeStamps();
		m_CurrStamp = 0;
	}

	SquareRange GetRange(const Tobject& obj) const
	{
		SquareRange r;

		auto& aabb = obj.GetWorldAABB();
		auto c = aabb.GetCentre();
		auto e = aabb.GetExtends();

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

	const GridSquaresBucket<Tobject>& GetBucket(int bucketIndex)
	{
		return GetBucketLocal(bucketIndex);
	}

	void Clear()
	{
		m_CurrStamp++;
		if (m_CurrStamp == std::numeric_limits<uint64>::max())
		{
			ClearTimeStamps();
			m_CurrStamp = 1;
		}
	}

	void Insert(Tobject& toInsert)
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
	std::vector<GridSquaresBucket<Tobject>> m_Buckets;
	std::vector<uint64> m_TimeStamps;

	float m_SquareSize;
	uint64 m_CurrStamp;
};