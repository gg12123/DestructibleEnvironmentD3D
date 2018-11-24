#pragma once

#include "Vector3.h"
#include "FinalFaceCreator.h"
#include "Transform.h"
#include "NewPointsGetter.h"
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

	int GetRequiredVertexCount() const
	{
		return m_RequiredNumVerts;
	}

	int GetRequiredIndexCount() const
	{
		return m_RequiredNumIndicies;
	}

	void InitRequiredVertAndIndexCounts();

	void UpdateWorldBounds(RadiusBoundsType)
	{
		m_WorldBounds.ConstrcutFromCentreAndRadius(m_Transform.GetPosition(), m_BoundingRadius);
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
	}

	void AddFace(Face& f)
	{
		m_Faces.emplace_back(&f);
	}

	void OnAllFacesAdded(Transform& refTran) // faces are in the ref transforms local space
	{
		CollectPointsAndEdges();

		auto c = CalculateCentre();

		ReCentre(c);
		InitFaces();

		GetTransform().SetPosition(refTran.ToWorldPosition(c));
		GetTransform().SetRotation(refTran.GetRotation());

		SetDirty();
	}

	// Call this one when re-centring is not required
	void OnAllFacesAdded()
	{
		CollectPointsAndEdges();
		InitFaces();
		SetDirty();
	}

private:
	void SetDirty()
	{
		m_Dirty = true;
	}

	Vector3 CalculateSplitPlaneNormal(const Vector3& P0);
	Vector3 CalculateCentre();
	void ReCentre(const Vector3& centre);
	void CollectPointsAndEdges();
	void TryCollectPoint(ShapePoint& p);
	void TryCollectEdge(ShapeEdge& p);
	void InitFaces();

	int m_RequiredNumVerts;
	int m_RequiredNumIndicies;

	std::vector<Face*> m_Faces;
	std::vector<Vector3> m_CachedPoints;
	std::vector<ShapePoint*> m_PointObjects;
	std::vector<ShapeEdge*> m_EdgeObjects;
	std::vector<int> m_EdgeIndexes;

	Transform m_Transform;
	Bounds m_LocalBounds;
	Bounds m_WorldBounds;

	bool m_Dirty = true;

	// float m_TotalEdgeLength = 0.0f; // TODO - work this back in somehow
	float m_BoundingRadius;
};