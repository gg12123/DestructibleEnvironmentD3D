#pragma once
#include "ShapePoint.h"
#include "ShapeEdge.h"
#include "Face.h"
#include "Vector3.h"
#include "ShapeElementPool.h"
#include "Shape.h"

class PiercedOnlyFaceHandler
{
private:
	void CreateCentreEdges(ShapePoint& pC)
	{
		m_CentreEdges.clear();
		for (auto p : m_OuterPoints)
			m_CentreEdges.emplace_back(EdgePool::Take(pC, *p));
	}

	template<class T>
	void CopyIntoVector(const std::vector<T*>& src, std::vector<T*>& dest)
	{
		dest.clear();
		dest.insert(dest.begin(), src.begin(), src.end());
	}

	void AddPointsAndEdgesToFace(Face& face, ShapePoint& pC, int i)
	{
		auto m = CollectionU::GetNextIndex(m_OuterPoints, i);

		face.AddPoint(*m_OuterPoints[i], *m_OuterEdges[i]);
		face.AddPoint(*m_OuterPoints[m], *m_CentreEdges[m]);
		face.AddPoint(pC, *m_CentreEdges[i]);
	}

public:
	struct PiercedFace
	{
		Face* TheFace;
		Vector3 CentreOfInters;

		PiercedFace(Face& f, const Vector3& c) : CentreOfInters(c), TheFace(&f)
		{
		}
	};

	void Handle(const PiercedFace& face)
	{
		auto& f = *face.TheFace;
		auto& s = f.GetOwnerShape();
		auto& pC = PointPool::Take(face.CentreOfInters);

		CopyIntoVector(f.GetPointObjects(), m_OuterPoints);
		CopyIntoVector(f.GetEdgeObjects(), m_OuterEdges);

		CreateCentreEdges(pC);

		f.Clear();
		AddPointsAndEdgesToFace(f, pC, 0);

		for (auto i = 1u; i < m_OuterPoints.size(); i++)
		{
			auto& fNew = FacePool::Take();
			AddPointsAndEdgesToFace(fNew, pC, i);
			fNew.SetNormal(f.GetNormal(), f.GetPlaneId());
			s.AddFace(fNew);
		}

		s.AddPoint(pC);
		for (auto e : m_CentreEdges)
			s.AddEdge(*e);
	}

private:
	std::vector<ShapeEdge*> m_CentreEdges;

	std::vector<ShapePoint*> m_OuterPoints;
	std::vector<ShapeEdge*> m_OuterEdges;
};