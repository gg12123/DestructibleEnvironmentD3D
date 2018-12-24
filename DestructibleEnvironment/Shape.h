#pragma once

#include "Vector3.h"
#include "Transform.h"
#include "Bounds.h"
#include "PointInPolyCase.h"
#include <vector>

class ShapePoint;
class ShapeEdge;
class Face;

class Shape
{
public:
	Shape()
	{
	}

	virtual ~Shape();

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

	auto& GetLocalBounds()
	{
		return m_LocalBounds;
	}

	auto& GetWorldBounds()
	{
		return m_WorldBounds;
	}

	Transform& GetTransform()
	{
		return m_Transform;
	}

	bool IsDirty()
	{
		return m_Dirty;
	}

	void ClearDirty()
	{
		m_Dirty = false;
	}

	void UpdateWorldBounds(RadiusBoundsType)
	{
	}

	void UpdateWorldBounds(AABBBoundsType)
	{
	}

	void Clear()
	{
		m_Faces.clear();
		m_CachedPoints.clear();
		m_EdgeObjects.clear();
		m_EdgeIndexes.clear();
		m_PointObjects.clear();
		m_LocalBounds.Reset();
	}

	void AddFace(Face& f)
	{
		m_Faces.emplace_back(&f);
	}

	void OnAllFacesAdded(Transform& refTran) // faces are in the ref transforms local space
	{
		RemoveSmallEdges(); // May also need to remove faces that have long edges but small area.
		CollectPointsAndEdges();

		auto c = CalculateCentre();

		ReCentre(c);
		InitFacesPointsEdges();

		GetTransform().SetPosition(refTran.ToWorldPosition(c));
		GetTransform().SetRotation(refTran.GetRotation());

		SetDirty();
	}

	// For when the centre is at the local origin.
	void OnAllFacesAdded()
	{
		CollectPointsAndEdges();

		ReCentre(Vector3::Zero());
		InitFacesPointsEdges();

		SetDirty();
	}

private:
	void SetDirty()
	{
		m_Dirty = true;
	}

	Vector3 CalculateCentre();
	void ReCentre(const Vector3& centre);
	void CollectPointsAndEdges();
	void RemoveSmallEdges();
	void TryCollectPoint(ShapePoint& p);
	void TryCollectEdge(ShapeEdge& p);
	void InitFacesPointsEdges();

	std::vector<Face*> m_Faces;
	std::vector<Vector3> m_CachedPoints;
	std::vector<ShapePoint*> m_PointObjects;
	std::vector<ShapeEdge*> m_EdgeObjects;
	std::vector<int> m_EdgeIndexes;

	Transform m_Transform;
	Bounds m_LocalBounds;
	Bounds m_WorldBounds;

	bool m_Dirty = true;
};