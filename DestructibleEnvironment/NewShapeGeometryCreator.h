#pragma once
#include "IntersectionLoop.h"
#include "MapToNewEdges.h"
#include "ShapePoint.h"
#include "MapToPointPlaneRelationship.h"
#include "ShapeElementPool.h"

class MapToNewPoints
{
private:
	struct PointPair
	{
		ShapePoint* Above;
		ShapePoint* Below;

		PointPair(ShapePoint& a, ShapePoint& b) : Above(&a), Below(&b)
		{
		}
	};

public:
	ShapePoint & GetPointAbove(const ShapeEdge& splitEdge) const
	{
		return *m_Points[splitEdge.GetHash()].Above;
	}

	ShapePoint & GetPointBelow(const ShapeEdge& splitEdge) const
	{
		return *m_Points[splitEdge.GetHash()].Below;
	}

	void RegisterNewPoints(ShapeEdge& splitEdge, ShapePoint& above, ShapePoint& below)
	{
		splitEdge.TryAssignHash();
		m_Points[splitEdge.GetHash()] = PointPair(above, below);
	}

	template<PointPlaneRelationship r>
	ShapePoint & GetPoint(const ShapeEdge& splitEdge) const
	{
		return GetPointAbove(splitEdge);
	}

	template<>
	ShapePoint & GetPoint<PointPlaneRelationship::PointsBelow>(const ShapeEdge& splitEdge) const
	{
		return GetPointBelow(splitEdge);
	}

private:
	DynamicArray<PointPair> m_Points;
};

class NewShapeGeometryCreator
{
private:
	void CreateEdgesAlongLoop(const EdgeFaceIntersection& inter1, const EdgeFaceIntersection& inter2)
	{
		auto& e1 = inter1.GetEdge();
		auto& e2 = inter2.GetEdge();

		auto& edgeAbove = EdgePool::Take(m_MapToPoints.GetPointAbove(e1), m_MapToPoints.GetPointAbove(e2));
		auto& edgeBelow = EdgePool::Take(m_MapToPoints.GetPointBelow(e1), m_MapToPoints.GetPointBelow(e2));

		m_MapToEdges.AddNewEdge(edgeAbove);
		m_MapToEdges.AddNewEdge(edgeAbove);
	}

	void CreateEdgesAlongSplitEdge(const EdgeFaceIntersection& inter)
	{
		auto& se = inter.GetEdge();

		auto& existingAbove = m_MapToPointPlane->GetRelationship(se.GetP0()) == PointPlaneRelationship::PointsAbove ?
			se.GetP0() : se.GetP1();

		auto& existingBelow = se.GetOther(existingAbove);

		auto& edgeAbove = EdgePool::Take(existingAbove, m_MapToPoints.GetPointAbove(se));
		auto& edgeBelow = EdgePool::Take(existingBelow, m_MapToPoints.GetPointBelow(se));

		m_MapToEdges.AddNewEdge(edgeAbove);
		m_MapToEdges.AddNewEdge(edgeAbove);
	}

	void CreateNewPoints(const IntersectionLoop& loop)
	{
		for (auto i = 0; i < loop.GetCount(); i++)
		{
			auto intPoint = loop.GetIntPoint(i);
			m_MapToPoints.RegisterNewPoints(loop.GetPiercingEdge(i), PointPool::Take(intPoint), PointPool::Take(intPoint));
		}
	}

	void CreateNewEdges(const IntersectionLoop& loop)
	{
		auto& inters = loop.GetIntersections();
		auto count = loop.GetCount();

		for (auto i = 0; i < count; i++)
		{
			CreateEdgesAlongSplitEdge(inters[i]);
			CreateEdgesAlongLoop(inters[i], inters[(i + 1) % count]);
		}
	}

public:
	void Init(MapToPointPlaneRelationship& pointPlaneMap)
	{
		m_MapToPointPlane = &pointPlaneMap;
	}

	void CreateGeometry(const IntersectionLoop& loop)
	{
		CreateNewPoints(loop);
		CreateNewEdges(loop);
	}

	const auto& GetNewEdgeMap() const
	{
		return m_MapToEdges;
	}

	const auto& GetNewPointMap() const
	{
		return m_MapToPoints;
	}

private:
	MapToNewEdges m_MapToEdges;
	MapToNewPoints m_MapToPoints;
	MapToPointPlaneRelationship* m_MapToPointPlane;
};