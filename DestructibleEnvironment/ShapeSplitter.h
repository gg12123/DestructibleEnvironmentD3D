#pragma once
#include <vector>
#include "Shape.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "ShapePoint.h"
#include "FaceSplitter.h"
#include "CleanIntersectionFinder.h"
#include "FaceIterator.h"
#include "ShapeElementPool.h"
#include "RedundantPointRemover.h"
#include "FaceTriangulator.h"
#include "NewShapeGeometryCreator.h"
#include "InPlaneFacesCreator.h"
#include "Random.h"

template<class Tshape>
class ShapeSplitter
{
private:
	void ResetHashCounters()
	{
		ObjectWithHash<Face>::ResetNextHashCounter();
		ObjectWithHash<ShapePoint>::ResetNextHashCounter();
		ObjectWithHash<ShapeEdge>::ResetNextHashCounter();
	}

	void CreateNewShapes(Tshape& original, std::vector<Tshape*>& newAboveShapes, std::vector<Tshape*>& newBelowShapes)
	{
		m_FaceIterator.SetShapeToUseNext(original);
		m_FaceIterator.CreateShapes(m_NewAboveFaces, newAboveShapes);
		m_FaceIterator.CreateShapes(m_NewBelowFaces, newBelowShapes);
	}

	void CleanGeometry(const std::vector<Tshape*>& newShapes) const
	{
		// TODO
	}

	void InitNewShapes(const std::vector<Tshape*>& newShapes)
	{
		for (auto s : newShapes)
			s->CollectShapeElementsAndResetHashes();
	}

	bool FindIntersections(const Shape& originalShape, const Plane& sp)
	{
		m_Intersections.clear();
		return m_IntersectionFinder.FindCleanIntersections(originalShape, sp, m_Intersections);
	}

	void CreateNewGeometry()
	{
		m_NewGeometryCreator.Init(m_IntersectionFinder.GetPointPlaneMap());

		for (auto l : m_Intersections)
			m_NewGeometryCreator.CreateGeometry(*l);
	}

	void SplitFaces()
	{
		m_FaceSplitter.Init(m_NewGeometryCreator.GetNewPointMap(), m_NewGeometryCreator.GetNewEdgeMap(), m_IntersectionFinder.GetPointPlaneMap());

		m_NewAboveFaces.clear();
		m_NewBelowFaces.clear();

		for (auto l : m_Intersections)
			m_FaceSplitter.Split(*l, m_NewAboveFaces, m_NewBelowFaces);
	}

	void CreateInPlaneFaces(const Plane& sp)
	{
		m_InPlaneFaceCreator.Init(m_NewGeometryCreator.GetNewPointMap(), m_NewGeometryCreator.GetNewEdgeMap());

		for (auto l : m_Intersections)
			m_InPlaneFaceCreator.Create(*l, sp, m_NewAboveFaces, m_NewBelowFaces);
	}

	void ReturnStuffToPool()
	{
		for (auto l : m_Intersections)
		{
			for (auto i = 0; i < l->GetCount(); i++)
			{
				FacePool::Return(l->GetFaceEntered(i));
				EdgePool::Return(l->GetPiercingEdge(i));
			}
		}
	}

public:
	bool Split(const Plane& sp, Tshape& originalShape, std::vector<Tshape*>& newShapesAbove, std::vector<Tshape*>& newShapesBelow)
	{
		ResetHashCounters();
		// Assume hashes on all shape elements are reset.

		if (!FindIntersections(originalShape, sp))
		{
			// log error
			return false;
		}

		CreateNewGeometry();
		SplitFaces();
		CreateInPlaneFaces(sp);
		CreateNewShapes(originalShape, newShapesAbove, newShapesBelow);

		CleanGeometry(newShapesAbove);
		CleanGeometry(newShapesBelow);

		InitNewShapes(newShapesAbove);
		InitNewShapes(newShapesBelow);

		ReturnStuffToPool();
		return true;
	}

private:
	FaceSplitter m_FaceSplitter;
	FaceIterator<Tshape> m_FaceIterator;
	CleanIntersectionFinder m_IntersectionFinder;
	InPlaneFaceCreator m_InPlaneFaceCreator;
	NewShapeGeometryCreator m_NewGeometryCreator;

	std::vector<IntersectionLoop*> m_Intersections;

	std::vector<Face*> m_NewAboveFaces;
	std::vector<Face*> m_NewBelowFaces;
};
