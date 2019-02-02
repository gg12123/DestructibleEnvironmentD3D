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
#include "ShapeElementPool.h"
#include "ShapeEdgesCreator.h"

template<class T>
class Triangulator
{
private:
	int GetNext(int curr) const
	{
		return CollectionU::GetNextIndex(m_CurrPoly, curr);
	}

	int GetPrevious(int curr) const
	{
		return CollectionU::GetPrevIndex(m_CurrPoly, curr);
	}

	const Vector2& GetPoint(int indexInCurr) const
	{
		return m_AllPoints[m_CurrPoly[indexInCurr]];
	}

	bool IsInsideCorner(const Vector2& prev, const Vector2& corner, const Vector2& next) const
	{
		auto v0 = corner - prev;
		auto v1 = next - corner;

		auto v0Mag = v0.Magnitude();
		auto v1Mag = v1.Magnitude();

		// TODO - this is a bit flaky
		if (v0Mag <= MathU::Epsilon || v1Mag <= MathU::Epsilon)
			return true;

		v0 /= v0Mag;
		v1 /= v1Mag;

		return Vector2::Cross2D(v0, v1) >= 0.0f;
	}

	bool TriangleCanBeClipped(int corner)
	{
		auto prev = GetPrevious(corner);
		auto next = GetNext(corner);

		m_Poly.Clear();
		m_Poly.Add(GetPoint(prev));
		m_Poly.Add(GetPoint(corner));
		m_Poly.Add(GetPoint(next));

		if (!IsInsideCorner(m_Poly.GetPointAt(0), m_Poly.GetPointAt(1), m_Poly.GetPointAt(2)))
			return false;

		for (auto i = GetNext(next); i != prev; i = GetNext(i))
		{
			auto& p = GetPoint(i);

			// Use the winding method because it works when points are coincident (I think).
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

class FaceTriangulator
{
private:
	const auto& OriginalPoints() const
	{
		return m_OriginalFace->GetPointObjects();
	}

	const auto& OriginalEdges() const
	{
		return m_OriginalFace->GetEdgeObjects();
	}

	ShapeEdge& GetEdgeBetween(int i0, int i1)
	{
		auto& points = OriginalPoints();
		auto& edges = OriginalEdges();

		if (i0 == CollectionU::GetPrevIndex(points, i1))
			return *edges[i0];

		if (i1 == CollectionU::GetPrevIndex(points, i0))
			return *edges[i1];

		auto& p0 = *points[i0];
		auto& p1 = *points[i1];

		if (!m_NewEdges->EdgeExistsBetween(p0, p1))
			m_NewEdges->CreateEdge(p0, p1);

		return m_NewEdges->GetMapToNewEdges().GetNewEdge(p0, p1);
	}

	Face& CreateFace(int i0, int i1, int i2)
	{
		auto& f = FacePool::Take();
		auto& points = OriginalPoints();

		f.AddPoint(*points[i0], GetEdgeBetween(i0, i1));
		f.AddPoint(*points[i1], GetEdgeBetween(i1, i2));
		f.AddPoint(*points[i2], GetEdgeBetween(i2, i0));

		f.SetNormal(m_OriginalFace->GetNormal(), m_OriginalFace->GetPlaneId());

		return f;
	}

public:
	void Init(ShapeEdgesCreator& edges)
	{
		m_NewEdges = &edges;
	}

	void Triangulate(const Face& originalFace, std::vector<Face*>& triangleFaces)
	{
		m_OriginalFace = &originalFace;
		m_TriangleFaces = &triangleFaces;

		for (auto e : m_OriginalFace->GetEdgeObjects())
			e->DeRegisterFace(*m_OriginalFace);

		m_Triangulator.Clear();
		for (auto p : m_OriginalFace->GetPointObjects())
			m_Triangulator.AddPoint(m_OriginalFace->ToFaceSpacePosition(p->GetPoint()));

		m_Triangulator.Triangulate(*this);
	}

	void OnTriangle(int i0, int i1, int i2)
	{
		m_TriangleFaces->emplace_back(&CreateFace(i0, i1, i2));
	}

private:
	ShapeEdgesCreator * m_NewEdges = nullptr;

	std::vector<Face*>* m_TriangleFaces;
	Triangulator<FaceTriangulator> m_Triangulator;

	const Face* m_OriginalFace = nullptr;
};
