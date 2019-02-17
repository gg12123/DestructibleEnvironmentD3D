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

class ShapeSplitter
{
private:
	void ResetHashCounters()
	{
		ObjectWithHash<Face>::ResetNextHashCounter();
		ObjectWithHash<ShapePoint>::ResetNextHashCounter();
		ObjectWithHash<ShapeEdge>::ResetNextHashCounter();
	}

	void CreateNewShapes(Shape& original, std::vector<Shape*>& newAboveShapes, std::vector<Shape*>& newBelowShapes)
	{
		m_FaceIterator.SetShapeToUseNext(original);
		m_FaceIterator.CreateShapes(m_NewAboveFaces, newAboveShapes);
		m_FaceIterator.CreateShapes(m_NewBelowFaces, newBelowShapes);
	}

	void CleanGeometry(const std::vector<Shape*>& newShapes) const
	{
		// TODO
	}

	void InitNewShapes(const std::vector<Shape*>& newShapes)
	{
		for (auto s : newShapes)
			s->CollectShapeElementsAndResetHashes();
	}

	CleanIntersectionFinder::Result FindIntersections(const Shape& originalShape, const Plane& sp)
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

	void ResetPointAndFaceHashes(const Shape& s) const
	{
		ObjectWithHash<ShapePoint>::ResetHashes(s.GetPointObjects());
		ObjectWithHash<Face>::ResetHashes(s.GetFaces());
	}

public:
	bool Split(const Plane& sp, Shape& originalShape, std::vector<Shape*>& newShapesAbove, std::vector<Shape*>& newShapesBelow)
	{
		ResetHashCounters();
		// Assume hashes on all shape elements are reset.

		auto interRes = FindIntersections(originalShape, sp);

		if (interRes == CleanIntersectionFinder::Result::Error)
		{
			// log error
			return false;
		}
		else if (interRes == CleanIntersectionFinder::Result::IntersectionsFound)
		{
			CreateNewGeometry();
			SplitFaces();
			CreateInPlaneFaces(sp);
			CreateNewShapes(originalShape, newShapesAbove, newShapesBelow);

			CleanGeometry(newShapesAbove);
			CleanGeometry(newShapesBelow);

			// These calls will reset the hashes
			InitNewShapes(newShapesAbove);
			InitNewShapes(newShapesBelow);

			ReturnStuffToPool();
		}
		else
		{
			ResetPointAndFaceHashes(originalShape);
			if (interRes == CleanIntersectionFinder::Result::ShapesAllAbove)
				newShapesAbove.emplace_back(&originalShape);
			else
				newShapesBelow.emplace_back(&originalShape);
		}
		return true;
	}

private:
	FaceSplitter m_FaceSplitter;
	FaceIterator m_FaceIterator;
	CleanIntersectionFinder m_IntersectionFinder;
	InPlaneFaceCreator m_InPlaneFaceCreator;
	NewShapeGeometryCreator m_NewGeometryCreator;

	std::vector<IntersectionLoop*> m_Intersections;

	std::vector<Face*> m_NewAboveFaces;
	std::vector<Face*> m_NewBelowFaces;
};
