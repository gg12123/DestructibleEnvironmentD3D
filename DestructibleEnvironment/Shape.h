#pragma once

#include "Vector.h"
#include "FinalFaceCreator.h"
#include "Transform.h"
#include "NewPointsGetter.h"
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

	std::vector<Point*>& GetPoints()
	{
		return m_Points;
	}

	std::vector<ShapeEdge*>& GetEdges()
	{
		return m_Edges;
	}

	std::vector<Face*>& GetFaces()
	{
		return m_Faces;
	}

	std::vector<Vector3>& GetCachedPoints()
	{
		return m_CachedPoints;
	}

	std::vector<Vector3>& GetCachedEdgePoints()
	{
		return m_CachedEdgePoints;
	}

	void Clear()
	{
		m_Faces.clear();
		m_Points.clear();
		m_Edges.clear();

		m_CachedPoints.clear();
		m_CachedEdgePoints.clear();

		m_CurrId = 0;
	}

	void AddNewEdgeFromFaceSplit(ShapeEdge& e)
	{
		m_Edges.push_back(&e);
		m_FinalFaceCreator.AddEdge(e);
	}

	Transform& GetTransform()
	{
		return m_Transform;
	}

	void AddPoint(Point& p);

	bool Split(const Vector3& collPointWs, const Vector3& collNormalWs, Shape& shapeAbove);

	bool IsDirty()
	{
		return m_Dirty;
	}

	void SetDirty()
	{
		m_Dirty = true;
	}

	void ClearDirty()
	{
		m_Dirty = false;
	}

	int GetRequiredVertexCount() const;
	int GetRequiredIndexCount() const;

private:

	Vector3 CalculateSplitPlaneNormal(const Vector3& P0, const Vector3& collNormal);
	Vector3 CentreAndCache();
	Vector3 CalculateCentre();
	void InitFaces(const Vector3& finalFaceNormal);
	void InitNewShape(Shape& shape, const Vector3& finalFaceNormal);
	bool SplitPoints(const Vector3& P0, const Vector3& n, Shape& shapeAbove, Shape& shapeBelow);

	std::vector<Point*> m_Points;
	std::vector<ShapeEdge*> m_Edges;
	std::vector<Face*> m_Faces;

	std::vector<Vector3> m_CachedPoints;
	std::vector<Vector3> m_CachedEdgePoints;

	static NewPointsGetter m_NewPointsGetter;

	FinalFaceCreator m_FinalFaceCreator;
	Transform m_Transform;

	int m_CurrId = 0;
	bool m_Dirty = true;
};