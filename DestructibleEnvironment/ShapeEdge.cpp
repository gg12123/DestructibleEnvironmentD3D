// Fill out your copyright notice in the Description page of Project Settings.

#include "ShapeEdge.h"
#include "Point.h"
#include "Shape.h"
#include "NewPointsGetter.h"

ShapeEdge::ShapeEdge()
{
}

ShapeEdge::~ShapeEdge()
{
}

void ShapeEdge::Cache(std::vector<Vector3>& edgePoints)
{
	edgePoints.push_back(m_EdgeP1->GetPoint());
	edgePoints.push_back(m_EdgeP2->GetPoint());
}

void ShapeEdge::SplitAtEnd(NewPointsGetter& newPoints, Shape& shapeAbove, Shape& shapeBelow)
{
	if (m_EdgeP1->GetPlaneRelationship() == PointPlaneRelationship::Inside)
	{
		if (m_EdgeP2->GetPlaneRelationship() == PointPlaneRelationship::Above)
		{
			m_EdgeP1 = &newPoints.GetPointAbove(*m_EdgeP1);
			shapeAbove.GetEdges().push_back(this);
		}
		else
		{
			m_EdgeP1 = &newPoints.GetPointBelow(*m_EdgeP1);
			shapeBelow.GetEdges().push_back(this);
		}
	}
	else if (m_EdgeP2->GetPlaneRelationship() == PointPlaneRelationship::Inside)
	{
		if (m_EdgeP1->GetPlaneRelationship() == PointPlaneRelationship::Above)
		{
			m_EdgeP2 = &newPoints.GetPointAbove(*m_EdgeP2);
			shapeAbove.GetEdges().push_back(this);
		}
		else
		{
			m_EdgeP2 = &newPoints.GetPointBelow(*m_EdgeP2);
			shapeBelow.GetEdges().push_back(this);
		}
	}
	else
	{
		assert(false);
	}
}

void ShapeEdge::SplitInHalf(NewPointsGetter& newPoints, const Vector3& x, Shape& shapeAbove, Shape& shapeBelow)
{
	// get from pool
	auto a = new Point(x);
	auto b = new Point(x);

	newPoints.AddPoints(*m_EdgeP1, *m_EdgeP2, *a, *b);

	shapeAbove.AddPoint(*a);
	shapeBelow.AddPoint(*b);

	if (m_EdgeP1->GetPlaneRelationship() == PointPlaneRelationship::Above)
	{
		auto newForBelow = new ShapeEdge(*m_EdgeP2, *b);
		m_EdgeP2 = a;

		shapeAbove.GetEdges().push_back(this);
		shapeBelow.GetEdges().push_back(newForBelow);
	}
	else
	{
		auto newForAbove = new ShapeEdge(*m_EdgeP2, *a);

		m_EdgeP2 = b;

		shapeAbove.GetEdges().push_back(newForAbove);
		shapeBelow.GetEdges().push_back(this);
	}
}

void ShapeEdge::Split(const Vector3& P0, const Vector3& n, NewPointsGetter& newPoints, Shape& shapeAbove, Shape& shapeBelow)
{
	if (Point::PointsBridgePlane(*m_EdgeP1, *m_EdgeP2))
	{
		auto x = Vector3::LinePlaneIntersection(P0, n, m_EdgeP1->GetPoint(), m_EdgeP2->GetPoint());

		SplitInHalf(newPoints, x, shapeAbove, shapeBelow);
	}
	else if (Point::BothAbove(*m_EdgeP1, *m_EdgeP2))
	{
		shapeAbove.GetEdges().push_back(this);
	}
	else if (Point::BothBelow(*m_EdgeP1, *m_EdgeP2))
	{
		shapeBelow.GetEdges().push_back(this);
	}
	else if (Point::BothInside(*m_EdgeP1, *m_EdgeP2))
	{
		// do nothing - new edges for each shape will be created by the face split
	}
	else
	{
		SplitAtEnd(newPoints, shapeAbove, shapeBelow);
	}
}
