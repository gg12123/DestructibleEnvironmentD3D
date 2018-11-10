// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vector3.h"
#include "Vector2.h"
#include "FaceFaceIntersection.h"
#include "Polygon2.h"
#include "PoolOfRecyclables.h"
#include "ObjectWithHash.h"
#include <vector>
#include <memory>

class Shape;
class ShapeEdge;
class ShapePoint;

struct FaceEdgeCaseResult
{
	ShapeEdge* Edge;
	Vector3 IntPoint;
	float Distance;
};

/**
 * 
 */
class Face : public ObjectWithHash<Face>
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

	void AddPoint(ShapePoint& point, const Vector3& dirToNext, ShapeEdge& edgeToNext)
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

	Vector3 GetEdgeNormal(const ShapeEdge& edge) const;

	FaceEdgeCaseResult CastToEdgeInside(const Vector3& origin, const Vector3& dir);

	Vector3 GetNormalWorld() const
	{
	}

	auto GetPlaneP0() const
	{
		return m_CachedPoints[0];
	}

	const auto& GetCachedPoints() const
	{
		return m_CachedPoints;
	}

	const auto& GetPointObjects() const
	{
		return m_PointObjects;
	}

	const auto& GetEdgeObjects() const
	{
		return m_EdgeObjects;
	}

	const auto& GetEdgeDirections() const
	{
		return m_EdgeDirections;
	}

	Shape& GetShape() const
	{
		return *m_OwnerShape;
	}

	const auto& GetFacePoly()
	{
		return m_FacePoly;
	}

	const auto& GetSharedPoints() const
	{
		return m_ToSharedPoints;
	}

	void Clear()
	{

	}

	bool PointIsInsideFace(const Vector3& pointShapesSpace) const;

	int NextPointIndex(int index) const
	{

	}

private:
	void InitFaceCoOrdinateSystem(const Vector3& origin);


	// all points related collections must be parralel.

	std::vector<int> m_ToSharedPoints;
	std::vector<Vector3> m_CachedPoints;
	std::vector<Vector3> m_EdgeDirections;
	std::vector<ShapePoint*> m_PointObjects;
	std::vector<ShapeEdge*> m_EdgeObjects;

	Shape* m_OwnerShape;
	Polygon2 m_FacePoly;

	Vector3 m_Normal;
};
