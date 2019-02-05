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

	void CreateNewShapes(Tshape& original, std::vector<Tshape*>& newShapes)
	{
		m_FaceIterator.SetShapeToUseNext(original);
		m_FaceIterator.CreateShapes(m_NewAboveFaces, newShapes);
		m_FaceIterator.CreateShapes(m_NewBelowFaces, newShapes);
	}

	void InitNewShapes(Shape& original, const std::vector<Tshape*>& newShapes, int nextPlaneId)
	{
		// Must copy the transform.
		// Copy is needed becasue the original shape is re-used in the
		// new shapes.
		auto t = original.GetTransform();

		for (auto it = newShapes.begin(); it != newShapes.end(); it++)
			(*it)->OnAllFacesAdded(t, nextPlaneId);
	}

	bool FindIntersections(const Shape& originalShape, const Plane& sp)
	{
		m_Intersections.clear();
		return m_IntersectionFinder.FindCleanIntersections(originalShape, sp, m_Intersections);
	}

	void CreateNewGeometry()
	{
		m_NewGeometryCreator.Init()

		for (auto l : m_Intersections)
			m_NewGeometryCreator.CreateGeometry(*l);
	}

	void RemoveRedundantPoints()
	{
		m_RedundantPointsRemover.Init(m_EdgesCreator, m_Reverser.GetMapToReversed());

		for (auto l : m_Intersections)
			m_RedundantPointsRemover.RemovePoints(*l);
	}

	void SplitFaces()
	{
		m_FaceSplitter.Init();

		m_NewAboveFaces.clear();
		m_NewBelowFaces.clear();

		for (auto l : m_Intersections)
			m_FaceSplitter.Split(*l, m_NewAboveFaces, m_NewBelowFaces);
	}

	void CreateInPlaneFaces(int planeId, const Plane& sp)
	{
		for (auto l : m_Intersections)
			m_InPlaneFaceCreator.Create(*l, sp, m_NewAboveFaces, m_NewBelowFaces, planeId);
	}

	void TriangulateShapesFaces(const std::vector<Tshape*>& newShapes)
	{
		m_Triangulator.Init(m_EdgesCreator);
		for (auto s : newShapes)
			s->TriangulateFaces(m_Triangulator);
	}

	int NextPlaneId(const Shape& originalShape) const
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

	Plane CalculateSplitPlane(const Vector3& splitPointLocal) const
	{
		auto a = Random::Range(0.01f, 1.0f);
		auto b = Random::Range(0.01f, 1.0f);
		auto c = Random::Range(0.01f, 1.0f);

		a *= Random::Range(0.0f, 1.0f) > 0.5f ? -1.0f : 1.0f;
		b *= Random::Range(0.0f, 1.0f) > 0.5f ? -1.0f : 1.0f;
		c *= Random::Range(0.0f, 1.0f) > 0.5f ? -1.0f : 1.0f;

		return Plane(Vector3(a, b, c).Normalized(), splitPointLocal);
	}

public:
	bool Split(const Vector3& splitPointWorld, Tshape& originalShape, std::vector<Tshape*>& newShapes)
	{
		auto planeId = NextPlaneId(originalShape);
		auto plane = CalculateSplitPlane(originalShape.GetTransform().ToLocalPosition(splitPointWorld));

		if (!FindIntersections(originalShape, plane))
		{
			// log error
			return false;
		}

		ResetHashCounters();

		CreateNewGeometry();
		SplitFaces();
		CreateInPlaneFaces(planeId);
		CreateNewShapes(originalShape, newShapes);

		RemoveRedundantPoints();
		TriangulateShapesFaces(newShapes);
		InitNewShapes(originalShape, newShapes, planeId + 1);

		return true;
	}

private:
	FaceSplitter m_FaceSplitter;
	FaceIterator<Tshape> m_FaceIterator;
	RedundantPointRemover m_RedundantPointsRemover;
	FaceTriangulator m_Triangulator;
	CleanIntersectionFinder m_IntersectionFinder;
	InPlaneFaceCreator m_InPlaneFaceCreator;
	NewShapeGeometryCreator m_NewGeometryCreator;

	std::vector<IntersectionLoop*> m_Intersections;

	std::vector<Face*> m_NewAboveFaces;
	std::vector<Face*> m_NewBelowFaces;
};
