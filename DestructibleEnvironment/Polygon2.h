#pragma once
#include <vector>
#include "Vector2.h"
#include "ReadOnlyView.h"

class Polygon2
{
public:

	bool PointIsInsideAssumingConvex(const Vector2& p) const
	{

	}

	auto GetPointCount() const
	{
		return m_Points.size();
	}

	auto GetPointAt(int index) const
	{
		return m_Points[index];
	}

	auto GetPoints() const
	{
		return ReadOnlyView<Vector2>(m_Points);
	}

	void Clear()
	{
		m_Points.clear();
	}

	void Add(const Vector2& p)
	{
		m_Points.emplace_back(p);
	}

	int NextIndex(int index) const
	{
		return (index + 1) % m_Points.size();
	}

private:
	std::vector<Vector2> m_Points;
};