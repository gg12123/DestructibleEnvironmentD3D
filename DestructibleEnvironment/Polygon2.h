#pragma once
#include <vector>
#include "Vector2.h"
#include "PointInPolyCase.h"
#include "MathU.h"

class EdgeCastHit
{
public:
	Vector2 HitPoint;
	int HitEdgeIndex;

	EdgeCastHit(const Vector2& hp, int i)
	{
		HitPoint = hp;
		HitEdgeIndex = i;
	}
};

class Polygon2
{
public:

	bool PointIsInsideConvexMethod(const Vector2& p) const
	{
		assert(false);
		return false;
	}

	bool PointIsInsideWindingMethod(const Vector2& p) const
	{
		auto wn = 0;
		auto c = m_Points.size();

		for (auto i = 0U; i < c; i++)
		{
			auto next = (i + 1) % c;
			auto amountLeft = AmountLeft(m_Points[i], m_Points[next], p);

			if ((m_Points[next].y  > p.y) && (m_Points[i].y <= p.y)) // upward crossing
			{ 
				if (amountLeft >= 0.0f)
					wn++;
			}
			else if ((m_Points[next].y <= p.y) && (m_Points[i].y > p.y)) // downward crossing
			{
				if (amountLeft < 0.0f)
					wn--;
			}
		}

		return (wn != 0);
	}

	auto GetPointCount() const
	{
		return m_Points.size();
	}

	auto GetPointAt(int index) const
	{
		return m_Points[index];
	}

	Vector2 GetNormalAt(int index) const
	{
		auto d = GetDirectionAt(index);
		return Vector2(-d.y, d.x); // TODO - is this the right rotation direction?
	}

	Vector2 GetDirectionAt(int index) const
	{
		return (m_Points[NextIndex(index)] - m_Points[index]).Normalized();
	}

	const auto& GetPoints() const
	{
		return m_Points;
	}

	auto GetCentre() const
	{
		return m_SumOfPoints / static_cast<float>(m_Points.size());
	}

	void Clear()
	{
		m_Points.clear();
		m_SumOfPoints = Vector2::Zero();
	}

	void Add(const Vector2& p)
	{
		m_Points.emplace_back(p);
		m_SumOfPoints += p;
	}

	int NextIndex(int index) const
	{
		return (index + 1) % m_Points.size();
	}

	int PreviousIndex(int index) const
	{
		auto s = m_Points.size();
		return (index - 1 + s) % s;
	}

private:
	static inline float AmountLeft(const Vector2& lineP0, const Vector2& lineP1, const Vector2& p)
	{
		return ((lineP1.x - lineP0.x) * (p.y - lineP0.y) - (p.x - lineP0.x) * (lineP1.y - lineP0.y));
	}

	std::vector<Vector2> m_Points;
	Vector2 m_SumOfPoints;
};