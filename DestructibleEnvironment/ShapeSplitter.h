#pragma once
#include <vector>
#include "Shape.h"
#include "Face.h"
#include "ShapeEdge.h"
#include "ShapePoint.h"
#include "CutShapeCreator.h"
#include "ShapeEdgesCreator.h"
#include "FaceSplitter.h"
#include "CutPathCreator.h"
#include "CleanIntersectionFinder.h"
#include "FaceIterator.h"
#include "ReversedGeometryCreator.h"
#include "ShapeElementPool.h"
#include "RedundantPointRemover.h"
#include "FaceTriangulator.h"
#include "PiercedOnlyFaceHandler.h"

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

	void CreateEdgesFromCPs()
	{
		auto& cps = m_CutPathCreator.GetCutPaths();

		for (auto it = cps.Begin(); it != cps.End(); it++)
			m_EdgesCreator.CreateEdgesAlongCutPath(*it);
	}

	void CreateInsideEdgesFromSplitEdges()
	{
		m_EdgesCreator.CreateInsideEdges();
	}

	void CreateOutsideEdgesFromSplitEdges()
	{
		m_EdgesCreator.CreateOutsideEdges(m_Reverser.GetMapToReversed());
	}

	void CreateNewInsideFaces(const Shape& cutShape)
	{
		m_NewInsideFaces.clear();
		m_NewInsideFacesFromCutShape.clear();

		auto& fcpColls = m_CutPathCreator.GetFacesCutPathCollections();

		for (auto it = fcpColls.Begin(); it != fcpColls.End(); it++)
		{
			auto& fcpColl = *it;
			auto sizeBefore = m_NewInsideFaces.size();
			m_FaceSplitter.SpitInside(fcpColl, m_NewInsideFaces);

			if (&fcpColl.GetFace().GetOwnerShape() == &cutShape)
			{
				for (auto i = sizeBefore; i < m_NewInsideFaces.size(); i++)
					m_NewInsideFacesFromCutShape.emplace_back(m_NewInsideFaces[i]);
			}
		}
	}

	void InitFaceSplitter()
	{
		m_FaceSplitter.InitMaps(m_Reverser.GetMapToReversed(), m_EdgesCreator, m_CutPathCreator.GetMapToFacesCutPaths());
	}

	void CreateNewOutsideFaces(const Shape& originalShape)
	{
		m_NewOutsideFaces.clear();

		auto& fcpColls = m_CutPathCreator.GetFacesCutPathCollections();

		for (auto it = fcpColls.Begin(); it != fcpColls.End(); it++)
		{
			auto& fcpColl = *it;
			if (&fcpColl.GetFace().GetOwnerShape() == &originalShape)
				m_FaceSplitter.SplitOutside(fcpColl, m_NewOutsideFaces);
		}
	}

	void CreateNewInsideShapes(Tshape& original, std::vector<Tshape*>& newShapes)
	{
		m_FaceIterator.SetShapeToUseNext(original);
		m_FaceIterator.CreateShapes(m_NewInsideFaces, newShapes, FaceRelationshipWithOtherShape::InIntersection);
	}

	void CreateNewOutsideShapes(std::vector<Tshape*>& newShapes)
	{
		m_FaceIterator.CreateShapes(m_NewOutsideFaces, newShapes, FaceRelationshipWithOtherShape::NotInIntersection);
	}

	void CreateReversedGeometry(const Shape& cutShape)
	{
		m_Reverser.Init(m_EdgesCreator);

		auto& cps = m_CutPathCreator.GetCutPaths();
		for (auto it = cps.Begin(); it != cps.End(); it++)
			m_Reverser.CreateReversedCutPath(*it);

		for (auto it = m_NewInsideFacesFromCutShape.begin(); it != m_NewInsideFacesFromCutShape.end(); it++)
			m_Reverser.CreateReversedFace(**it);

		auto& faces = cutShape.GetFaces();
		for (auto it = faces.begin(); it != faces.end(); it++)
		{
			auto& f = **it;
			if (f.HashIsAssigned() && !m_CutPathCreator.FaceIsSplit(f))
			{
				// If the hash is assiged and the face is not split, it means that the hash was
				// assigned by the face iterator when it created the inside shape. Hence the face
				// is inside and needs reversing.

				m_Reverser.CreateReversedFace(f);
			}
		}
	}

	void InitNewShapes(Shape& original, std::vector<Tshape*>& newShapes, int nextPlaneId)
	{
		// Must copy the transform.
		// Copy is needed becasue the original shape is re-used in the
		// new shapes.
		auto t = original.GetTransform();

		for (auto it = newShapes.begin(); it != newShapes.end(); it++)
			(*it)->OnAllFacesAdded(t, nextPlaneId);
	}

	bool FindIntersections(const Shape& originalShape, const Shape& cutShape)
	{
		m_Intersections.clear();
		return m_IntersectionFinder.FindCleanIntersections(cutShape, originalShape, m_Intersections);
	}

	void CollectFacesAndEdgesForPool(const Shape& originalShape)
	{
		m_OriginalShapeFacesForPool.clear();
		for (auto f : originalShape.GetFaces())
		{
			if (m_CutPathCreator.FaceIsSplit(*f))
				m_OriginalShapeFacesForPool.emplace_back(f);
		}

		m_OriginalShapeEdgesForPool.clear();
		for (auto e : originalShape.GetEdgeObjects())
		{
			if (e->IsSplit())
				m_OriginalShapeEdgesForPool.emplace_back(e);
		}
	}

	void ReturnUnusedOriginalShapeStuffToPool()
	{
		for (auto f : m_OriginalShapeFacesForPool)
			FacePool::Return(*f);

		for (auto e : m_OriginalShapeEdgesForPool)
			EdgePool::Return(*e);
	}

	template<class T>
	void ReturnIfStillOwnedByCutShape(const std::vector<T*>& objs, const Shape& cutShape)
	{
		for (auto o : objs)
		{
			if (&o->GetOwnerShape() == &cutShape)
				ShapeElementPool<T>::Return(*o);
		}
	}

	void ReturnUnusedCutShapeStuffToPool(const Shape& cutShape)
	{
		ReturnIfStillOwnedByCutShape(cutShape.GetEdgeObjects(), cutShape);
		ReturnIfStillOwnedByCutShape(cutShape.GetPointObjects(), cutShape);
		ReturnIfStillOwnedByCutShape(cutShape.GetFaces(), cutShape);
	}

	int TotalIntersectionCount() const
	{
		auto c = 0;
		for (auto loop : m_Intersections)
			c += loop->GetCount();
		return c;
	}

	void RemoveRedundantPoints()
	{
		m_RedundantPointsRemover.Init(m_EdgesCreator, m_Reverser.GetMapToReversed());

		auto& cps = m_CutPathCreator.GetCutPaths();
		for (auto it = cps.Begin(); it != cps.End(); it++)
			m_RedundantPointsRemover.RemovePoints(*it);
	}

	void TriangulateShapesFaces(const std::vector<Tshape*>& newShapes)
	{
		m_Triangulator.Init(m_EdgesCreator);
		for (auto s : newShapes)
			s->TriangulateFaces(m_Triangulator);
	}

	int FirstPlaneIdForCutShape(const Shape& originalShape)
	{
		auto maxId = -1;
		for (auto f : originalShape.GetFaces())
		{
			auto id = f->GetPlaneId();
			if (id > maxId)
				maxId = id;
		}
		return maxId + 1;
	}

	void GenerateCutPaths(const Shape& originalShape, const Shape& cutShape)
	{
		auto success = false;

		while (!success)
		{
			// TODO - abort nicely if this returns false.
			assert(FindIntersections(originalShape, cutShape));

			// The intersection finder uses the hashes so reset the counters here ready
			// for the rest of the splitting algorithm.
			ResetHashCounters();

			PiercedOnlyFaceHandler::PiercedFace piercedOnlyFace;
			success = m_CutPathCreator.GeneratePaths(m_Intersections, piercedOnlyFace);

			if (!success)
				m_PiercedOnlyFaceHandler.Handle(piercedOnlyFace);
		}
	}

public:
	void Split(const Vector3& splitPointWorld, Tshape& originalShape, std::vector<Tshape*>& newShapes)
	{
		auto nextPlaneId = FirstPlaneIdForCutShape(originalShape);
		auto& cutShape = m_CutShapeCreator.Create(originalShape, splitPointWorld, nextPlaneId);

		nextPlaneId += CutShapeCreator::NumPlanesInCutShape;

		GenerateCutPaths(originalShape, cutShape);

		m_EdgesCreator.Init(TotalIntersectionCount());
		CreateEdgesFromCPs();
		CreateInsideEdgesFromSplitEdges();

		// Now get the stuff from original shape that needs returning to the pool. But dont return
		// it until the end of the split. Returning it early may break some state that is relied
		// upon in the rest of the algorithm.
		CollectFacesAndEdgesForPool(originalShape);

		InitFaceSplitter();

		CreateNewInsideFaces(cutShape);
		CreateNewInsideShapes(originalShape, newShapes);

		CreateReversedGeometry(cutShape);
		CreateOutsideEdgesFromSplitEdges();

		CreateNewOutsideFaces(originalShape);
		CreateNewOutsideShapes(newShapes);

		RemoveRedundantPoints();

		TriangulateShapesFaces(newShapes);
		InitNewShapes(originalShape, newShapes, nextPlaneId);

		// To clean up, loop through everything on the cut shape and if it has not
		// been assigned to one of the new shapes (check owner shape property), it must be
		// returned to the pool. Also any faces and edges on the original shape that got split
		// must be returned. All points on the original shape are re-used. To check if an
		// edge is split, use the reference to split edge. To check if a face is split, use
		// the cut path creator.
		// Cut shape doesnt need returning to the pool becasue it is re-used in the cut shape
		// creator.
		ReturnUnusedCutShapeStuffToPool(cutShape);
		ReturnUnusedOriginalShapeStuffToPool();

		// No need to un-assign any hashes because that is done by the new
		// shapes in 'on splitting finished' and when an object is taken from
		// a pool.
	}

private:
	CutShapeCreator m_CutShapeCreator;
	FaceSplitter m_FaceSplitter;
	ShapeEdgesCreator m_EdgesCreator;
	CutPathCreator m_CutPathCreator;
	FaceIterator<Tshape> m_FaceIterator;
	ReversedGeometryCreator m_Reverser;
	RedundantPointRemover m_RedundantPointsRemover;
	FaceTriangulator m_Triangulator;
	PiercedOnlyFaceHandler m_PiercedOnlyFaceHandler;

	CleanIntersectionFinder m_IntersectionFinder;
	std::vector<IntersectionLoop*> m_Intersections;

	std::vector<Face*> m_NewInsideFacesFromCutShape;

	std::vector<Face*> m_NewInsideFaces;
	std::vector<Face*> m_NewOutsideFaces;

	std::vector<Face*> m_OriginalShapeFacesForPool;
	std::vector<ShapeEdge*> m_OriginalShapeEdgesForPool;
};
