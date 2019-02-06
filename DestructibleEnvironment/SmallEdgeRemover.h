#pragma once
#include "ShapeEdge.h"
#include "Face.h"
#include "ShapePoint.h"
#include "IterationAboutShape.h"
#include "ShapeElementPool.h"

class SmallEdgeRemover
{
private:
	void CollapseFace(Face& f, const ShapeEdge& smallEdge)
	{
		auto& edges = f.GetEdgeObjects();

		auto smallEdgeIndexInFace = smallEdge.GetIndex(f);

		auto& e0 = *edges[f.PreviousPointIndex(smallEdgeIndexInFace)];
		auto& e1 = *edges[f.NextPointIndex(smallEdgeIndexInFace)];

		auto& e0SideFace = e0.GetOther(f);

		e1.DeRegisterFace(f);
		e0SideFace.ReplaceEdge(e0, e1);

		EdgePool::Return(e0);
		FacePool::Return(f);
	}

public:
	void RemoveEdge(ShapeEdge& toRemove)
	{
		m_EdgesAboutPoint0.clear();
		m_EdgesAboutPoint1.clear();
		m_FacesAboutPoints.clear();

		auto& p0 = toRemove.GetP0();
		auto& p1 = toRemove.GetP1();

		IterationAboutShape::FindEdgesAndFacesAboutPoint(p0, toRemove, m_EdgesAboutPoint0, m_FacesAboutPoints);
		IterationAboutShape::FindEdgesAndFacesAboutPoint(p1, toRemove, m_EdgesAboutPoint1, m_FacesAboutPoints);

		auto& newPoint = PointPool::Take((p0.GetPoint() + p1.GetPoint()) / 2.0f);

		// Make all the edges reference the new point

		for (auto it = m_EdgesAboutPoint0.begin(); it != m_EdgesAboutPoint0.end(); it++)
			(*it)->ReplacePoint(p0, newPoint);

		for (auto it = m_EdgesAboutPoint1.begin(); it != m_EdgesAboutPoint1.end(); it++)
			(*it)->ReplacePoint(p1, newPoint);

		// Collapse the faces either side of the small edge

		auto removedFace1 = &toRemove.GetFace1();
		auto removedFace2 = &toRemove.GetFace2();

		CollapseFace(toRemove.GetFace1(), toRemove);
		CollapseFace(toRemove.GetFace2(), toRemove);

		// Make the faces grab the new point and re-calculate normals
		for (auto it = m_FacesAboutPoints.begin(); it != m_FacesAboutPoints.end(); it++)
		{
			auto f = *it;

			if (f != removedFace1 && f != removedFace2)
			{
				f->ReplacePointObjects(p0, p1, newPoint);
			}
		}

		PointPool::Return(p0);
		PointPool::Return(p1);
	}

private:
	std::vector<ShapeEdge*> m_EdgesAboutPoint0;
	std::vector<ShapeEdge*> m_EdgesAboutPoint1;
	std::vector<Face*> m_FacesAboutPoints;
};
