// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <vector>
#include <memory>

#include "CollectionU.h"
#include "Vector3.h"
#include "Vector2.h"
#include "Polygon2.h"
#include "ObjectWithHash.h"
#include "Plane.h"

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
	Plane ToPlane() const
	{
		return Plane(m_Normal, m_CachedPoints[0]);
	}

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

	void AddPoint(ShapePoint& point, ShapeEdge& edgeToNext);

	void SetNormal(const Vector3& normal, int planeId)
	{
		assert(planeId >= 0);
		m_Normal = normal;
		m_PlaneId = planeId;
	}

	void CalculateNormalFromPoints(int planeId);

	auto GetPlaneId() const
	{
		return m_PlaneId;
	}

	auto GetNormal() const
	{
		return m_Normal;
	}

	Vector3 GetEdgeNormal(const ShapeEdge& edge) const;
	Vector3 GetEdgeNormal(int index) const;

	Vector3 GetEdgeDirection(const ShapeEdge& edge) const;
	Vector3 GetEdgeDirection(int index) const;

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

	Shape& GetOwnerShape() const
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
		return CollectionU::GetNextIndex(m_PointObjects, index);
	}

	int PreviousPointIndex(int index) const
	{
		return CollectionU::GetPrevIndex(m_PointObjects, index);
	}

	void OnSplittingFinished(Shape& owner);

	void Clear()
	{
		m_CachedPoints.clear();
		m_PointObjects.clear();
		m_EdgeObjects.clear();
		m_FacePoly.Clear();
	}

	void OnTakenFromPool()
	{
		m_OwnerShape = nullptr;
		ResetHash();
		Clear();
	}

	void MergeWith(const Face& other, const ShapeEdge& commonEdge);
	void RemovePoint(const ShapePoint& toRemove, ShapeEdge& newEdge);
	void ReplaceEdge(const ShapeEdge& existing, ShapeEdge& newEdge);
	void ReplacePointObjects(const ShapePoint& oldP0, const ShapePoint& oldP1, ShapePoint& replacement);

	void InitFaceCoOrdinateSystem();

private:
	void InitFaceCoOrdinateSystem(const Vector3& origin)
	{
		m_FaceSpace.Init(m_Normal, origin);
	}

	// all points related collections must be parralel.
	std::vector<Vector3> m_CachedPoints;
	std::vector<ShapePoint*> m_PointObjects;
	std::vector<ShapeEdge*> m_EdgeObjects;

	Shape* m_OwnerShape = nullptr;
	Polygon2 m_FacePoly;

	Vector3 m_Normal;
	int m_PlaneId;

	FaceCoOrdinateSystem m_FaceSpace;
};
