// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <vector>
#include <memory>

#include "CollectionU.h"
#include "Vector3.h"
#include "Vector2.h"
#include "Polygon2.h"
#include "ObjectWithHash.h"

class Shape;
class ShapeEdge;
class ShapePoint;

struct FaceEdgeCaseResult
{
	ShapeEdge* Edge;
	Vector3 IntPoint;
	float Distance;

	FaceEdgeCaseResult(ShapeEdge& edge, const Vector3 intPoint, float dist) : IntPoint(intPoint), Distance(dist)
	{
		Edge = &edge;
	}

	FaceEdgeCaseResult() : IntPoint(Vector3::Zero()), Distance(MathU::Infinity)
	{
		Edge = nullptr;
	}
};

class FaceCoOrdinateSystem
{
public:
	void Init(const Vector3& faceNormal, const Vector3& pointOnFace)
	{
		m_Origin = pointOnFace;
		m_XDir = Vector3::OrthogonalDirection(faceNormal);
		m_YDir = Vector3::Cross(faceNormal, m_XDir);
	}

	Vector2 ToFaceSpaceDirection(const Vector3& shapesSpaceDir) const
	{
		return Vector2(Vector3::Dot(shapesSpaceDir, m_XDir), Vector3::Dot(shapesSpaceDir, m_YDir));
	}

	Vector2 ToFaceSpacePosition(const Vector3& shapesSpacePos) const
	{
		auto p = shapesSpacePos - m_Origin;
		return Vector2(Vector3::Dot(p, m_XDir), Vector3::Dot(p, m_YDir));
	}

	Vector3 ToShapeSpaceDirection(const Vector2& faceSpaceDir) const
	{
		return faceSpaceDir.x * m_XDir + faceSpaceDir.y * m_YDir;
	}

	Vector3 ToShapeSpacePosition(const Vector2& faceSpacePos) const
	{
		return m_Origin + faceSpacePos.x * m_XDir + faceSpacePos.y * m_YDir;
	}

private:
	Vector3 m_XDir;
	Vector3 m_YDir;
	Vector3 m_Origin;
};

/**
 * 
 */
class Face : public ObjectWithHash<Face>
{
public:
	Vector2 ToFaceSpaceDirection(const Vector3& shapesSpaceDir) const
	{
		return m_FaceSpace.ToFaceSpaceDirection(shapesSpaceDir);
	}

	Vector2 ToFaceSpacePosition(const Vector3& shapesSpacePos) const
	{
		return m_FaceSpace.ToFaceSpacePosition(shapesSpacePos);
	}

	Vector3 ToShapeSpaceDirection(const Vector2& faceSpaceDir) const
	{
		return m_FaceSpace.ToShapeSpaceDirection(faceSpaceDir);
	}

	Vector3 ToShapeSpacePosition(const Vector2& faceSpacePos) const
	{
		return m_FaceSpace.ToShapeSpacePosition(faceSpacePos);
	}

	void AddPoint(ShapePoint& point, const Vector3& dirToNext, ShapeEdge& edgeToNext);

	void SetNormal(const Vector3& normal)
	{
		m_Normal = normal;
	}

	auto GetNormal() const
	{
		return m_Normal;
	}

	Vector3 GetEdgeNormal(const ShapeEdge& edge) const;
	Vector3 GetEdgeNormal(int index) const;

	FaceEdgeCaseResult CastToEdgeInside(const Vector3& origin, const Vector3& dir);

	auto GetPlaneP0() const
	{
		return m_CachedPoints[0];
	}

	Vector3 GetCentre() const
	{
		return ToShapeSpacePosition(m_FacePoly.GetCentre());
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

	const auto& GetFacePoly() const
	{
		return m_FacePoly;
	}

	bool PointIsOnFace(const Vector3& pointShapesSpace) const;

	int NextPointIndex(int index) const
	{
		return CollectionU::GetNextIndex(m_CachedPoints, index);
	}

	int PreviousPointIndex(int index) const
	{
		return CollectionU::GetPrevIndex(m_CachedPoints, index);
	}

	void OnSplittingFinished(Shape& owner);

	void Clear()
	{
		m_CachedPoints.clear();
		m_EdgeDirections.clear();
		m_PointObjects.clear();
		m_EdgeObjects.clear();
		m_FacePoly.Clear();
	}

	void ReplaceEdge(const ShapeEdge& existing, ShapeEdge& newEdge)
	{
		auto it = std::find(m_EdgeObjects.begin(), m_EdgeObjects.end(), &existing);
		assert(it != m_EdgeObjects.end());
		*it = &newEdge;
	}

	void RefreshPointObjects();

private:
	void InitFaceCoOrdinateSystem(const Vector3& origin)
	{
		m_FaceSpace.Init(m_Normal, origin);
	}

	// all points related collections must be parralel.
	std::vector<Vector3> m_CachedPoints;
	std::vector<Vector3> m_EdgeDirections;
	std::vector<ShapePoint*> m_PointObjects;
	std::vector<ShapeEdge*> m_EdgeObjects;

	Shape* m_OwnerShape;
	Polygon2 m_FacePoly;

	Vector3 m_Normal;

	FaceCoOrdinateSystem m_FaceSpace;
};
