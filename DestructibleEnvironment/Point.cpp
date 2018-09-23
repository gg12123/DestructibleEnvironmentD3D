// Fill out your copyright notice in the Description page of Project Settings.
#include "pch.h"
#include "Point.h"
#include "Shape.h"
#include "NewPointsGetter.h"

void Point::Split(const Vector3& P0, const Vector3& n, NewPointsGetter& newPoints, Shape& shapeAbove, Shape& shapeBelow, int& countAbove, int& countBelow)
{
	m_LinkedPoint1 = nullptr;
	m_LinkedPoint2 = nullptr;
	m_OwnedEdgeLength = 0.0f;

	auto comp = Vector3::Dot(m_Point - P0, n);

	if (fabs(comp) <= PointInPlaneTol)
	{
		auto newAbove = this;
		auto newBelow = new Point(m_Point); // get from pool

		shapeAbove.AddPoint(*newAbove);
		shapeBelow.AddPoint(*newBelow);

		newPoints.AddPoints(*this, *newAbove, *newBelow);

		m_PlaneRelationship = PointPlaneRelationship::Inside;
	}
	else if (comp > 0.0f)
	{
		shapeAbove.AddPoint(*this);
		m_PlaneRelationship = PointPlaneRelationship::Above;
		countAbove++;
	}
	else
	{
		shapeBelow.AddPoint(*this);
		m_PlaneRelationship = PointPlaneRelationship::Below;
		countBelow++;
	}
}
