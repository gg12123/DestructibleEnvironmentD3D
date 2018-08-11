// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector.h"
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

	~Face()
	{
	}

	void SetNormal(const Vector3& normal)
	{
		m_Normal = normal;
	}

	void Split(const NewPointsGetter& newPoints, Shape& shapeAbove, Shape& shapeBelow);

	std::vector<Point*>& GetPoints()
	{
		return m_Points;
	}

	void CachePoints();
	int CountNumInside();

private:

	bool IsNewlyFormedEdge(const ShapeEdge& e) const;

	bool DefinesNewFace(const std::vector<Point*>& points) const
	{
		return (points.size() > 2);
	}

	void ProcessNewFace(Face& face, ShapeEdge& newEdge, Shape& shape) const;

	std::vector<Point*> m_Points;
	std::vector<Vector3> m_CachedPoints;

	Vector3 m_Normal;
};
