#pragma once

#include <vector>
#include "Vector3.h"
#include "Transform.h"
#include "Bounds.h"
#include "PointInPolyCase.h"
#include "RayCasting.h"
#include "LastCheckedAgainst.h"
#include "ObjectWithHash.h"
#include "CollectionU.h"

class ShapePoint;
class ShapeEdge;
class Face;
class CompoundShape;

class Shape : public LastCheckedAgainst<const Shape*>, public ObjectWithHash<Shape>
{
public:
	Shape()
	{
	}

	int GetShapeId() const
	{

	}

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

	const auto& GetEdgeIndexesPoints() const
	{
		return m_EdgeIndexesPoints;
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
		m_CachedFaceNormals.clear();
		m_EdgeObjects.clear();
		m_EdgeIndexesPoints.clear();
		m_EdgeIndexesFaces.clear();
		m_FaceP0Indexes.clear();
		m_PointObjects.clear();
		m_LinkedShapes.clear();
	}

	void AddLink(Shape& link)
	{
		m_LinkedShapes.emplace_back(&link);
		link.m_LinkedShapes.emplace_back(this);
	}

	void DisconnectLinks()
	{
		for (auto link : m_LinkedShapes)
			CollectionU::Remove(link->m_LinkedShapes, this);

		m_LinkedShapes.clear();
	}

	const auto& GetLinkedShapes() const
	{
		return m_LinkedShapes;
	}

	void AddFace(Face& f)
	{
		f.SetIndexInShape(m_Faces.size());
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

	void OnTakenFromPool()
	{
		Clear();
	}

	void OnReturnedToPool();

	Shape& Duplicate() const;

	void UpdateWorldAABB();

	const AABB& GetWorldAABB() const
	{
		return m_WorldAABB;
	}

private:
	void ReCentre(const Vector3& centre);
	void TryCollectPoint(ShapePoint& p);
	void TryCollectEdge(ShapeEdge& p);
	void InitFacesPointsEdges();

	std::vector<Face*> m_Faces;
	std::vector<ShapePoint*> m_PointObjects;
	std::vector<ShapeEdge*> m_EdgeObjects;

	std::vector<Vector3> m_CachedPoints;
	std::vector<Vector3> m_CachedFaceNormals;
	std::vector<int> m_FaceP0Indexes;
	std::vector<int> m_EdgeIndexesPoints;
	std::vector<int> m_EdgeIndexesFaces;
	Vector3 m_Centre;

	CompoundShape* m_Owner;

	AABB m_LocalAABB;
	AABB m_WorldAABB;

	std::vector<Shape*> m_LinkedShapes;
};