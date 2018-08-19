// Fill out your copyright notice in the Description page of Project Settings.
#include "pch.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "Shape.h"
#include "Point.h"
#include "NewPointsGetter.h"

int Face::CountNumInside()
{
	auto n = 0;
	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
	{
		if ((*it)->GetPlaneRelationship() == PointPlaneRelationship::Inside)
			n++;
	}
	return n;
}

void Face::Split(NewPointsGetter& newPoints, Shape& shapeAbove, Shape& shapeBelow)
{
	// use pool
	auto* aboveFace = new Face();
	auto* belowFace = new Face();

	auto& abovePoints = aboveFace->GetPoints();
	auto& belowPoints = belowFace->GetPoints();

	abovePoints.clear();
	belowPoints.clear();

	// use pool
	auto newAboveEdge = new ShapeEdge();
	auto newBelowEdge = new ShapeEdge();

	for (auto i = 0U; i < m_Points.size(); i++)
	{
		auto next = (i + 1) % m_Points.size();

		auto p1 = m_Points[i];
		auto p2 = m_Points[next];

		if (p1->GetPlaneRelationship() == PointPlaneRelationship::Above)
		{
			abovePoints.push_back(p1);
		}
		else if (p1->GetPlaneRelationship() == PointPlaneRelationship::Below)
		{
			belowPoints.push_back(p1);
		}
		else
		{
			auto a = &newPoints.GetPointAbove(*p1);
			auto b = &newPoints.GetPointBelow(*p1);

			abovePoints.push_back(a);
			belowPoints.push_back(b);

			newAboveEdge->AddPoint(*a);
			newBelowEdge->AddPoint(*b);
		}

		if (Point::PointsBridgePlane(*p1, *p2))
		{
			auto a = &newPoints.GetPointAbove(*p1, *p2);
			auto b = &newPoints.GetPointBelow(*p1, *p2);

			abovePoints.push_back(a);
			belowPoints.push_back(b);

			newAboveEdge->AddPoint(*a);
			newBelowEdge->AddPoint(*b);
		}
	}

	ProcessNewFace(*aboveFace, *newAboveEdge, shapeAbove);
	ProcessNewFace(*belowFace, *newBelowEdge, shapeBelow);

	// return this face to the pool
}

bool Face::IsNewlyFormedEdge(const ShapeEdge& e) const
{
	return (e.GetEdgeP1() != nullptr);
}

void Face::ProcessNewFace(Face& face, ShapeEdge& newEdge, Shape& shape) const
{
	if (DefinesNewFace(face.GetPoints()))
	{
		face.SetNormal(m_Normal);

		shape.GetFaces().push_back(&face);

		if (IsNewlyFormedEdge(newEdge))
			shape.AddNewEdgeFromFaceSplit(newEdge);
	}
	else
	{
		// return face to the pool
	}
}

void Face::CachePoints()
{
	m_CachedPoints.clear();

	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
		m_CachedPoints.push_back((*it)->GetPoint());
}
