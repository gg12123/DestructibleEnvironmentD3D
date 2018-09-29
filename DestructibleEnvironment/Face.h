// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
#include "FaceFaceIntersection.h"
#include "ReadOnlyView.h"
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

	void SetNormal(const Vector3& normal)
	{
		m_Normal = normal;
	}

	auto GetNormal() const
	{
		return m_Normal;
	}

	auto GetPlaneP0() const
	{
		return m_CachedPoints[0];
	}

	void RegisterIntersection(const FaceFaceIntersection& inter)
	{
		m_Intersections.emplace_back(inter);
	}

	auto GetIntersections()
	{
		return ReadOnlyView<std::vector<FaceFaceIntersection>>(m_Intersections);
	}

	auto GetCachedPoints()
	{
		return ReadOnlyView<std::vector<Vector3>>(m_CachedPoints);
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

	Vector3 m_Normal;
};
