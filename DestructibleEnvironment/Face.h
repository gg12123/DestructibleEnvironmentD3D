// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
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

	auto GetNormal() const
	{
		return m_Normal;
	}

	bool PreSplit(Shape& shapeAbove, Shape& shapeBelow, std::vector<Face*>& facesNeedingFullSplit);
	void Split(NewPointsGetter& newPoints, Shape& shapeAbove, Shape& shapeBelow);

	auto& GetPoints()
	{
		return m_Points;
	}

	auto& GetCachedPoints()
	{
		return m_CachedPoints;
	}

	void CachePoints(std::vector<Vector3>& faceNormals, std::vector<Vector3>& faceP0s);

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
