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

			if (&fcpColl.GetFace().GetShape() == &cutShape)
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
			if (&fcpColl.GetFace().GetShape() == &originalShape)
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

	void InitNewShapes(Shape& original, std::vector<Tshape*>& newShapes)
	{
		// Must copy the transform.
		// Copy is needed becasue the original shape is re-used in the
		// new shapes.
		auto t = original.GetTransform();

		for (auto it = newShapes.begin(); it != newShapes.end(); it++)
			(*it)->OnAllFacesAdded(t);
	}

	bool FindIntersections(const Shape& originalShape, const Shape& cutShape)
	{
		m_Intersections.clear();
		return m_IntersectionFinder.FindCleanIntersections(cutShape, originalShape, m_Intersections);
	}

public:
	void Split(const Vector3& splitPointWorld, const Vector3& splitNormalWorld, Tshape& originalShape, std::vector<Tshape*>& newShapes)
	{
		ResetHashCounters();

		auto& cutShape = m_CutShapeCreator.Create(originalShape.GetTransform(), splitPointWorld, splitNormalWorld);

		// TODO - abort nicely if this returns false.
		assert(FindIntersections(originalShape, cutShape));

		m_CutPathCreator.GeneratePaths(m_Intersections);

		CreateEdgesFromCPs();
		CreateInsideEdgesFromSplitEdges();

		// Now get the stuff from original shape that needs returning to the pool. But dont return
		// it until the end of the split.

		InitFaceSplitter();

		CreateNewInsideFaces(cutShape);
		CreateNewInsideShapes(originalShape, newShapes);

		CreateReversedGeometry(cutShape);
		CreateOutsideEdgesFromSplitEdges();

		CreateNewOutsideFaces(originalShape);
		CreateNewOutsideShapes(newShapes);

		InitNewShapes(originalShape, newShapes);

		// TODO - to clean up, loop through everything on the cut shape and if it has not
		// been assigned to one of the new shapes (check owner shape property), it must be
		// returned to the pool. Also any faces and edges on the original shape that got split
		// must be returned. All points on the original shape are re-used. To check if an
		// edge is split, use the reference to split edge. To check if a face is split, use
		// the cut path creator.
	}

private:
	CutShapeCreator m_CutShapeCreator;
	FaceSplitter m_FaceSplitter;
	ShapeEdgesCreator m_EdgesCreator;
	CutPathCreator m_CutPathCreator;
	FaceIterator<Tshape> m_FaceIterator;
	ReversedGeometryCreator m_Reverser;

	CleanIntersectionFinder m_IntersectionFinder;
	std::vector<IntersectionLoop*> m_Intersections;

	std::vector<Face*> m_NewInsideFacesFromCutShape;

	std::vector<Face*> m_NewInsideFaces;
	std::vector<Face*> m_NewOutsideFaces;
};
