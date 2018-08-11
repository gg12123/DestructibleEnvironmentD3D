// Fill out your copyright notice in the Description page of Project Settings.

#include "Point.h"
#include "Shape.h"
#include "NewPointsGetter.h"

void Point::Split(const Vector3& P0, const Vector3& n, NewPointsGetter& newPoints, Shape& shapeAbove, Shape& shapeBelow, int& numInside)
{
	m_LinkedPoint1 = nullptr;
	m_LinkedPoint2 = nullptr;

	auto comp = Vector3::Dot(m_Point - P0, n);

	if (fabs(comp) <= PointInPlaneTol)
	{
		auto newAbove = this;
		auto newBelow = new Point(m_Point); // get from pool

		newPoints.AddPoints(*this, *newAbove, *newBelow);

		shapeAbove.AddPoint(*newAbove);
		shapeBelow.AddPoint(*newBelow);

		m_PlaneRelationship = PointPlaneRelationship::Inside;
		numInside++;
	}
	else if (comp > 0.0f)
	{
		shapeAbove.AddPoint(*this);
		m_PlaneRelationship = PointPlaneRelationship::Above;
	}
	else
	{
		shapeBelow.AddPoint(*this);
		m_PlaneRelationship = PointPlaneRelationship::Below;
	}
}
