// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
#include "Vector2.h"
#include "FaceFaceIntersection.h"
#include "Polygon2.h"
#include <vector>

class Point;
class NewPointsGetter;
class Shape;
class ShapeEdge;

/**
 * 
 */
class Face
{
public:
	Face()
	{
		// allocate space in vectors
	}

	Vector2 ToFaceSpaceDirection(const Vector3& shapesSpaceDir) const
	{

	}

	Vector2 ToFaceSpacePosition(const Vector3& shapesSpacePos) const
	{

	}

	Vector3 ToShapeSpacePosition(const Vector2& faceSpacePos) const
	{

	}

	void SetNormal(const Vector3& normal)
	{
		m_Normal = normal;
	}

	auto GetNormal() const
	{
		return m_Normal;
	}

	Vector3 GetNormalWorld() const
	{
	}

	auto GetPlaneP0() const
	{
		return m_CachedPoints[0];
	}

	void RegisterIntersection(const FaceFaceIntersection& inter)
	{
		m_Intersections.emplace_back(inter);
	}

	const auto& GetIntersections()
	{
		return m_Intersections;
	}

	const auto& GetCachedPoints()
	{
		return m_CachedPoints;
	}

	auto& GetShape()
	{
		return *m_OwnerShape;
	}

	const auto& GetFacePoly()
	{
		return m_FacePoly;
	}

	void AddPoint(const Vector3& p)
	{
		m_CachedPoints.emplace_back(p);
	}

	void Clear()
	{
		m_ToSharedPoints.clear();
		m_CachedPoints.clear();
		m_Intersections.clear();
	}

	bool PointIsInsideFace(const Vector3& point);

private:
	std::vector<int> m_ToSharedPoints;
	std::vector<Vector3> m_CachedPoints;

	std::vector<FaceFaceIntersection> m_Intersections;
	Shape* m_OwnerShape;
	Polygon2 m_FacePoly;

	Vector3 m_Normal;
};
