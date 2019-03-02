#pragma once

#include <vector>
#include "Vector3.h"
#include "Transform.h"
#include "Bounds.h"
#include "PointInPolyCase.h"
#include "RayCasting.h"

class ShapePoint;
class ShapeEdge;
class Face;
class CompoundShape;

class Shape
{
public:
	Shape()
	{
	}

	virtual ~Shape();

	auto& GetOwner() const
	{
		return *m_Owner;
	}

	bool HasOwner() const
	{
		return m_Owner;
	}

	void SetOwner(CompoundShape& o)
	{
		m_Owner = &o;
	}

	const auto& GetFaces() const
	{
		return m_Faces;
	}

	const auto& GetCachedPoints() const
	{
		return m_CachedPoints;
	}

	const auto& GetEdgeIndexes() const
	{
		return m_EdgeIndexes;
	}

	const auto& GetEdgeObjects() const
	{
		return m_EdgeObjects;
	}

	const auto& GetPointObjects() const
	{
		return m_PointObjects;
	}

	void Clear()
	{
		m_Faces.clear();
		m_CachedPoints.clear();
		m_EdgeObjects.clear();
		m_EdgeIndexes.clear();
		m_PointObjects.clear();
	}

	void AddFace(Face& f)
	{
		m_Faces.emplace_back(&f);
	}

	void AddPoint(ShapePoint& p)
	{
		m_PointObjects.emplace_back(&p);
	}

	void AddEdge(ShapeEdge& e)
	{
		m_EdgeObjects.emplace_back(&e);
	}

	bool IntersectsRay(const Ray& localRay, Vector3& intPoint);

	// To create a new shape
	// - add all the faces
	// - call collect
	// - call 'centre and cache' (or call 'reset been collected' for intermediate shapes)

	void CollectShapeElementsAndResetHashes();
	void ResetBeenCollectedFlag();

	void CentreAndCache(const Vector3& ownersCentre)
	{
		ReCentre(ownersCentre);
		InitFacesPointsEdges();
	}

	Vector3 GetCentre() const
	{
		return m_Centre;
	}

	void CalculateCentre();

	void OnTakenFromPool()
	{
		Clear();
	}

	void OnReturnedToPool();

	Shape& Duplicate() const;

private:
	void ReCentre(const Vector3& centre);
	void TryCollectPoint(ShapePoint& p);
	void TryCollectEdge(ShapeEdge& p);
	void InitFacesPointsEdges();

	std::vector<Face*> m_Faces;
	std::vector<Vector3> m_CachedPoints;
	std::vector<ShapePoint*> m_PointObjects;
	std::vector<ShapeEdge*> m_EdgeObjects;
	std::vector<int> m_EdgeIndexes;
	Vector3 m_Centre;

	CompoundShape* m_Owner;
};