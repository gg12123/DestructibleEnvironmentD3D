#pragma once
#include <vector>
#include <array>
#include "Face.h"
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "Vector3.h"
#include "Vector2.h"
#include "Polygon2.h"
#include "CollectionU.h"

template<class T>
class Triangulator
{
private:
	int GetNext(int curr)
	{
		return CollectionU::GetNextIndex(m_CurrPoly, curr);
	}

	int GetPrevious(int curr)
	{
		return CollectionU::GetPrevIndex(m_CurrPoly, curr);
	}

	const Vector2& GetPoint(int indexInCurr)
	{
		return m_AllPoints[m_CurrPoly[indexInCurr]];
	}

	bool TriangleCanBeClipped(int corner)
	{
		auto prev = GetPrevious(corner);
		auto next = GetNext(corner);

		m_Poly.Clear();
		m_Poly.Add(GetPoint(prev));
		m_Poly.Add(GetPoint(corner));
		m_Poly.Add(GetPoint(next));

		for (auto i = GetNext(next); i != prev; i = GetNext(i))
		{
			auto& p = GetPoint(i);

			// Use the winding method because it works when points are coincident (I think),
			if (m_Poly.PointIsInsideWindingMethod(p))
				return false;
		}
		return true;
	}

	void Clip(int corner)
	{
		auto i0 = m_CurrPoly[GetPrevious(corner)];
		auto i1 = m_CurrPoly[corner];
		auto i2 = m_CurrPoly[GetNext(corner)];

		m_Responder->OnTriangle(i0, i1, i2);
	}

	int FindNextToClip()
	{
		for (auto i = 0U; i < m_CurrPoly.size(); i++)
		{
			if (TriangleCanBeClipped(i))
				return i;
		}
		assert(false);
		return -1;
	}

	void ClipNext()
	{
		auto toClip = FindNextToClip();

		Clip(toClip);
		m_CurrPoly.erase(m_CurrPoly.begin() + toClip);
	}

public:
	void Clear()
	{
		m_CurrPoly.clear();
		m_AllPoints.clear();
	}

	void AddPoint(const Vector2& p)
	{
		m_AllPoints.emplace_back(p);
		m_CurrPoly.emplace_back(m_CurrPoly.size());
	}

	void Triangulate(T& responder)
	{
		m_Responder = &responder;

		while (m_CurrPoly.size() > 3U)
			ClipNext();

		Clip(0);
	}

private:
	std::vector<Vector2> m_AllPoints;
	std::vector<int> m_CurrPoly;

	Polygon2 m_Poly;
	T* m_Responder;
};

class ConcaveFace
{
private:
	ShapeEdge& GetEdgeBetween(int i0, int i1)
	{
		if (i0 == CollectionU::GetPrevIndex(m_Points, i1))
			return *m_Edges[i0];

		if (i1 == CollectionU::GetPrevIndex(m_Points, i0))
			return *m_Edges[i1];

		auto& p0 = *m_Points[i0];
		auto& p1 = *m_Points[i1];

		if (!m_NewEdges->EdgeExistsBetween(p0, p1))
			m_NewEdges->CreateEdge(p0, p1);

		return m_NewEdges->GetMapToNewEdges().GetNewEdge(p0, p1);
	}

	Face& CreateFace(int i0, int i1, int i2)
	{
		auto& f = *(new Face()); // TODO - pool

		f.AddPoint(*m_Points[i0], GetEdgeBetween(i0, i1));
		f.AddPoint(*m_Points[i1], GetEdgeBetween(i1, i2));
		f.AddPoint(*m_Points[i2], GetEdgeBetween(i2, i0));

		f.SetNormal(m_Normal);

		return f;
	}

public:
	void Init(const Face& originalFace, ShapeEdgesCreator& edges)
	{
		m_Points.clear();
		m_Edges.clear();
		m_Triangulator.Clear();
		m_OriginalFace = &originalFace;
		m_NewEdges = &edges;
	}

	void AddPoint(ShapePoint& point, ShapeEdge& edgeToNext)
	{
		m_Points.emplace_back(&point);
		m_Edges.emplace_back(&edgeToNext);
		m_Triangulator.AddPoint(m_OriginalFace->ToFaceSpacePosition(point.GetPoint()));
	}

	void Triangulate(std::vector<Face*>& triangleFaces)
	{
		m_Normal = m_OriginalFace->GetNormal();
		m_TriangleFaces = &triangleFaces;
		m_Triangulator.Triangulate(*this);
	}

	void OnTriangle(int i0, int i1, int i2)
	{
		m_TriangleFaces->emplace_back(&CreateFace(i0, i1, i2));
	}

private:
	ShapeEdgesCreator * m_NewEdges = nullptr;

	std::vector<ShapePoint*> m_Points;
	std::vector<ShapeEdge*> m_Edges;

	std::vector<Face*>* m_TriangleFaces;
	Triangulator<ConcaveFace> m_Triangulator;

	Vector3 m_Normal;
	const Face* m_OriginalFace = nullptr;
};
