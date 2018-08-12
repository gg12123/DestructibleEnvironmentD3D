// Fill out your copyright notice in the Description page of Project Settings.
#include "pch.h"
#include "NewPointsGetter.h"
#include "Point.h"

Point& NewPointsGetter::GetPointAbove(const Point& existing1, const Point& existing2)
{
	return *Get(existing1, existing2).Above;
}

Point& NewPointsGetter::GetPointBelow(const Point& existing1, const Point& existing2)
{
	return *Get(existing1, existing2).Below;
}

Point& NewPointsGetter::GetPointAbove(const Point& inside)
{
	return *m_PointsOnPoints[inside.GetId()].Above;
}

Point& NewPointsGetter::GetPointBelow(const Point& inside)
{
	return *m_PointsOnPoints[inside.GetId()].Below;
}

void NewPointsGetter::AddPoints(const Point& existing1, const Point& existing2, Point& newAbove, Point& newBelow)
{
	Get(existing1, existing2).Set(&newAbove, &newBelow);
}

void NewPointsGetter::AddPoints(const Point& inside, Point& newAbove, Point& newBelow)
{
	m_PointsOnPoints[inside.GetId()].Set(&newAbove, &newBelow);
}

PointPair& NewPointsGetter::Get(const Point& existing1, const Point& existing2)
{
	if (existing1.GetPlaneRelationship() == PointPlaneRelationship::Above)
	{
		return m_PointsAlongEdges.Get(existing1.GetId(), existing2.GetId());
	}

	return m_PointsAlongEdges.Get(existing2.GetId(), existing1.GetId());
}
