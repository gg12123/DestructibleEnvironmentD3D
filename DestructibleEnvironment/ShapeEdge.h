// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector.h"
#include <assert.h>
#include <vector>

class Point;
class Shape;
class NewPointsGetter;

/**
 * 
 */
class ShapeEdge
{
public:
	ShapeEdge();

	ShapeEdge(Point& p1, Point& p2)
	{
		m_EdgeP1 = &p1;
		m_EdgeP2 = &p2;
	}

	~ShapeEdge();

	void AddPoint(Point &p)
	{
		if (m_EdgeP1 == nullptr)
		{
			m_EdgeP1 = &p;
		}
		else if (m_EdgeP2 == nullptr)
		{
			m_EdgeP2 = &p;
		}
		else
		{
			assert(false);
		}
	}

	void Cache(std::vector<Vector3>& edgePoints);

	void Split(const Vector3& P0, const Vector3& n, NewPointsGetter& newPoints, Shape& shapeAbove, Shape& shapeBelow);

	Point* GetEdgeP1() const
	{
		return m_EdgeP1;
	}

	Point* GetEdgeP2() const
	{
		return m_EdgeP1;
	}

private:
	void SplitInHalf(NewPointsGetter& newPoints, const Vector3& x, Shape& shapeAbove, Shape& shapeBelow);
	void SplitAtEnd(NewPointsGetter& newPoints, Shape& shapeAbove, Shape& shapeBelow);

	Point* m_EdgeP1;
	Point* m_EdgeP2;
};
