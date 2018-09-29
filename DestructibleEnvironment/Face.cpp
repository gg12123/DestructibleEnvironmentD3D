// Fill out your copyright notice in the Description page of Project Settings.
#include "pch.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "Shape.h"
#include "Point.h"
#include "NewPointsGetter.h"

bool Face::PreSplit(Shape& shapeAbove, Shape& shapeBelow, std::vector<Face*>& facesNeedingFullSplit)
{
	auto numInside = 0;
	auto numAbove = 0;
	auto numBelow = 0;

	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
	{
		switch ((*it)->GetPlaneRelationship())
		{
		case PointPlaneRelationship::Inside:
			numInside++;
			break;
		case PointPlaneRelationship::Above:
			numAbove++;
			break;
		case PointPlaneRelationship::Below:
			numBelow++;
			break;
		default:
			break;
		}
	}

	if (numInside == 3)
		return false;

	if ((numInside == 2) && (numAbove != 0) && (numBelow != 0))
		return false;

	if ((numInside == 2) || (numAbove != 0 && numBelow != 0))
	{
		facesNeedingFullSplit.emplace_back(this);
	}
	else if (numAbove == 0)
	{
		shapeBelow.GetFaces().emplace_back(this);
	}
	else if (numBelow == 0)
	{
		shapeAbove.GetFaces().emplace_back(this);
	}

	return true;
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

		shape.GetFaces().emplace_back(&face);

		if (IsNewlyFormedEdge(newEdge))
			shape.AddNewEdgeFromFaceSplit(newEdge);
	}
	else
	{
		// return face to the pool
	}
}

void Face::CachePoints(std::vector<Vector3>& faceNormals, std::vector<Vector3>& faceP0s)
{
	m_CachedPoints.clear();

	for (auto it = m_Points.begin(); it != m_Points.end(); it++)
		m_CachedPoints.emplace_back((*it)->GetPoint());

	faceNormals.emplace_back(m_Normal);
	faceP0s.emplace_back(m_CachedPoints[0]);
}

bool Face::PointIsInsideFace(const Vector3& point)
{
	auto size = m_CachedPoints.size();

	for (auto i = 0U; i < size; i++)
	{
		auto next = (i + 1U) % size;

		auto& P0 = m_CachedPoints[i];
		auto& P1 = m_CachedPoints[next];

		auto n = Vector3::Normalize(Vector3::Cross(P1 - P0, m_Normal));

		if (Vector3::Dot(n, point - P0); > 0.0f)
			return false;
	}
	return true;
}
