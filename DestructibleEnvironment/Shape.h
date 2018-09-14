#pragma once

#include "Vector3.h"
#include "FinalFaceCreator.h"
#include "Transform.h"
#include "NewPointsGetter.h"
#include "Bounds.h"
#include <vector>

class Point;
class ShapeEdge;
class Face;

class Shape
{
public:
	Shape()
	{
	}

	virtual ~Shape();

	auto& GetFaces()
	{
		return m_Faces;
	}

	auto& GetCachedPoints()
	{
		return m_CachedPoints;
	}

	auto& GetCachedEdgePoints()
	{
		return m_CachedEdgePoints;
	}

	auto& GetCachedFaceNormals()
	{
		return m_CachedFaceNormals;
	}

	auto& GetCachedFaceP0s()
	{
		return m_CachedFaceP0s;
	}

	auto& GetLocalBounds()
	{
		return m_LocalBounds;
	}

	void Clear()
	{
		m_Faces.clear();
		m_Points.clear();
		m_Edges.clear();

		m_CachedPoints.clear();
		m_CachedEdgePoints.clear();
		m_CachedFaceNormals.clear();
		m_CachedFaceP0s.clear();

		m_CurrId = 0;
		m_TotalEdgeLength = 0.0f;
	}

	void AddNewEdgeFromFaceSplit(ShapeEdge& e)
	{
		m_FinalFaceCreator.AddEdge(e);
		AddEdge(e);
	}

	Transform& GetTransform()
	{
		return m_Transform;
	}

	void AddPoint(Point& p);
	void AddEdge(ShapeEdge& e);

	bool Split(const Vector3& collPointWs, Shape& shapeAbove);

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

	float GetTotalEdgeLength() const
	{
		return m_TotalEdgeLength;
	}

	void InitRequiredVertAndIndexCounts();
	Vector3 CentreAndCache();

protected:
	auto& GetPoints()
	{
		return m_Points;
	}

private:
	void SetDirty()
	{
		m_Dirty = true;
	}

	Vector3 CalculateSplitPlaneNormal(const Vector3& P0);
	Vector3 CalculateCentre();
	void InitFaces(const Vector3& finalFaceNormal);
	void InitNewShape(Shape& shape, const Vector3& finalFaceNormal);
	bool SplitPoints(const Vector3& P0, const Vector3& n, Shape& shapeAbove, Shape& shapeBelow);
	void TransferSplitResultsToThis(Shape& splitResult);

	int m_RequiredNumVerts;
	int m_RequiredNumIndicies;

	std::vector<Point*> m_Points;
	std::vector<ShapeEdge*> m_Edges;
	std::vector<Face*> m_Faces;

	std::vector<Vector3> m_CachedPoints;
	std::vector<int> m_CachedEdgePoints;
	std::vector<Vector3> m_CachedFaceNormals;
	std::vector<Vector3> m_CachedFaceP0s;

	static NewPointsGetter m_NewPointsGetter;

	FinalFaceCreator m_FinalFaceCreator;
	Transform m_Transform;
	Bounds m_LocalBounds;

	int m_CurrId = 0;
	bool m_Dirty = true;
	float m_TotalEdgeLength = 0.0f;
};